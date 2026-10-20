#!/usr/bin/env python3
"""Aprende la compensacion de la retencion del motor (anti-cogging, M59) en lazo cerrado.

El firmware suma al adelanto del campo   tabla(fase) + s * tabla_sentido(fase),   con la fase = angulo
electrico absoluto del campo (64 puntos por ciclo) y s = velocidad del perfil / velocidad de
referencia, limitada a +-1 (0 quieto). Hacen falta las dos tablas porque el lazo responde distinto al
mismo patron segun el sentido de marcha: en la vuelta el patron pasa con el tiempo invertido y la
respuesta es la conjugada de la de la ida. Con una sola tabla, el promedio de ida y vuelta se anula
pero cada sentido queda con el mismo residuo y signo opuesto.

Cada iteracion hace una ida y vuelta 0 -> X_fin -> 0 a velocidad constante con telemetria y mide, en
cada sentido, el error de seguimiento plegado sobre el ciclo electrico (sin la media). Para cada
sentido d (+1 ida, -1 vuelta) la tabla efectiva es  T_d = tabla + d * tabla_sentido,  y se ajusta
armonico por armonico (h = 1..8) con el metodo de la secante:

  iteracion 0     tablas de partida (cero, o las de flash): referencia
  iteracion 1     prueba: tabla + 0,5 x patron del PI (para medir como responde cada armonico)
  iteraciones 2+  G_dh = (cambio del error) / (cambio de la tabla) con las dos ultimas pasadas,
                  T_dh <- T_dh - ganancia * error_dh / G_dh;  tabla = (T_ida + T_vuelta) / 2,
                  tabla_sentido = (T_ida - T_vuelta) / 2

Copiar directamente el patron del PI NO sirve: a 2 mm/s el paso del motor cae en 80 Hz, donde el lazo
responde con ~180 grados de atraso, y la tabla empuja para el mismo lado que la perturbacion (ver
docs/CAMBIOS_CALIBRACION_2026-10-20.md). Las tablas aprendidas valen para la velocidad a la que se
aprendieron (--feed).

Al final se carga la iteracion con menos error (RMS en movimiento); con --save se guarda en flash (M59 W).

Uso:
    python3 tools/cogging_learn.py                          # 0 -> 50 mm, referencia + 4 iteraciones
    python3 tools/cogging_learn.py --iters 0                # solo medir con las tablas actuales
    python3 tools/cogging_learn.py --continue-from mediciones/AAAA-MM-DD/cogging_HHMMSS --iters 3 --save
    python3 tools/cogging_learn.py --from-dir mediciones/AAAA-MM-DD/cogging_HHMMSS   # re-graficar

Requiere el eje homeado y calibrado, y el firmware con M59.
"""
import argparse
import datetime
import json
import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import closed_loop_test as T  # noqa: E402
import analisis_errores as A  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00"
N_TABLE = 64
POLE_PAIRS = 50
HARMONICS = range(1, 9)
MAX_TABLE_DEG = 20.0
DIRS = (+1, -1)


# ======================================================================================
# Plegado sobre el ciclo electrico
# ======================================================================================

def electrical_phase(pos_mm, field_at_zero):
    """Fase [0, 1) del angulo electrico del campo, el mismo indice que usa el firmware."""
    motor_rad = np.asarray(pos_mm) / T.PITCH_MM * 2 * np.pi
    f = (field_at_zero + motor_rad * POLE_PAIRS) / (2 * np.pi)
    return f - np.floor(f)


def fold(phase, y, n=N_TABLE):
    """Promedio de y en cada uno de los n intervalos de fase (centrados en i/n, como la tabla)."""
    b = np.floor(phase * n + 0.5).astype(int) % n
    cnt = np.bincount(b, minlength=n)
    s = np.bincount(b, y, minlength=n)
    return s / np.maximum(cnt, 1)


def folds_by_dir(d, meta, key):
    """{+1: patron de la ida, -1: patron de la vuelta} de d[key] sobre el ciclo electrico, sin la media."""
    out = {}
    for L in meta["legs"]:
        m = (d["leg"] == L["leg"]) & d["cruise"]
        if m.sum() < 2000:
            continue
        p = fold(electrical_phase(d["pos_mm"][m], meta["field_at_zero"]), d[key][m])
        out[L["dir"]] = p - p.mean()
    return out


def pi_pattern(d, meta):
    f = folds_by_dir(d, meta, "out_deg")
    return np.mean(list(f.values()), axis=0) if f else np.zeros(N_TABLE)


def err_fold(d, meta):
    f = folds_by_dir(d, meta, "err_um")
    return np.mean(list(f.values()), axis=0) if f else np.zeros(N_TABLE)


# ======================================================================================
# Aprendizaje
# ======================================================================================

def next_tables(history, gain, pi_last, G_mem):
    """Proximas (tabla, tabla_sentido) a partir de la historia [(tabla, tabla_sentido, {d: error}), ...]."""
    ts, ta, _ = history[-1]
    if len(history) == 1:
        return ts + 0.5 * pi_last, ta.copy()            # prueba
    ts_p, ta_p, e_p = history[-2]
    e_n = history[-1][2]
    new = {}
    for d in DIRS:
        Tn = np.fft.rfft(ts + d * ta)
        Tp = np.fft.rfft(ts_p + d * ta_p)
        En = np.fft.rfft(e_n[d])
        Ep = np.fft.rfft(e_p[d])
        out = Tn.copy()
        for h in HARMONICS:
            dt = Tn[h] - Tp[h]
            if abs(dt) * 2 / N_TABLE >= 0.2:            # el armonico cambio lo suficiente: estimar G
                G = (En[h] - Ep[h]) / dt
                if abs(G) > 1e-6:
                    G_mem[(d, h)] = G
            G = G_mem.get((d, h))
            if G is None and (-d, h) in G_mem:
                G = np.conj(G_mem[(-d, h)])              # la vuelta responde con la conjugada de la ida
            if G is None:
                continue
            out[h] = Tn[h] - gain * En[h] / G
        new[d] = np.fft.irfft(out, N_TABLE)
    ts_new = (new[+1] + new[-1]) / 2
    ta_new = (new[+1] - new[-1]) / 2
    peak = np.max(np.abs(ts_new) + np.abs(ta_new))
    if peak > MAX_TABLE_DEG:
        ts_new, ta_new = ts_new * MAX_TABLE_DEG / peak, ta_new * MAX_TABLE_DEG / peak
    return ts_new, ta_new


# ======================================================================================
# Comunicacion
# ======================================================================================

def m59(link, *args):
    return link.command("M59 " + " ".join(args) if args else "M59", echo=False)


def read_tables(link):
    lines = m59(link, "P")
    ts, ta = np.zeros(N_TABLE), np.zeros(N_TABLE)
    field = None
    for line in lines:
        mm = re.match(r"cog (\d+) ([-0-9.eE+]+) ([-0-9.eE+]+)", line)
        if mm:
            ts[int(mm.group(1))] = float(mm.group(2))
            ta[int(mm.group(1))] = float(mm.group(3))
        mf = re.search(r"campo_en_cero=([-0-9.eE+]+)", line)
        if mf:
            field = float(mf.group(1))
    if field is None:
        raise RuntimeError("M59 no respondio como se esperaba (¿firmware sin las dos tablas?)")
    return ts, ta, field


def upload(link, ts, ta, feed):
    m59(link, "C")
    for i in range(N_TABLE):
        if abs(ts[i]) > 1e-6:
            m59(link, f"I{i}", f"V{ts[i]:.4f}")
        if abs(ta[i]) > 1e-6:
            m59(link, f"J{i}", f"V{ta[i]:.4f}")
    enabled = bool(np.any(np.abs(ts) > 1e-6) or np.any(np.abs(ta) > 1e-6))
    m59(link, "G1", f"R{feed:g}", f"S{1 if enabled else 0}")
    return enabled


def run_pass(link, args, k):
    """Una ida y vuelta con reposos; devuelve la telemetria y los tramos."""
    link.rows.clear()
    link.leg.clear()
    link.current_leg = -1
    t0 = time.time()
    while time.time() - t0 < 0.5:
        link.poll()
    legs = []
    for j, (dest, d) in enumerate(((args.end, +1), (0.0, -1))):
        link.current_leg = j
        ok = T.run_move(link, dest, args.feed, 0.5, f"iter {k} {'ida' if d > 0 else 'vuelta'}", abort_err_um=150)
        legs.append({"leg": j, "name": "ida 1" if d > 0 else "vuelta 1", "dir": d, "cycle": 0, "dest": dest, "ok": ok})
        link.current_leg = -1
        t0 = time.time()
        while time.time() - t0 < args.dwell:
            link.poll()
        if not ok:
            break
    return list(link.rows), list(link.leg), legs


# ======================================================================================
# Metricas y graficos
# ======================================================================================

def metrics(d, meta):
    out = {}
    legs = [L for L in meta["legs"] if L["ok"]]
    mv = np.isin(d["leg"], [L["leg"] for L in legs])
    cr = mv & d["cruise"]
    out["rms"] = float(np.sqrt(np.mean(d["err_um"][cr] ** 2)))
    out["max"] = float(np.max(np.abs(d["err_um"][mv])))
    comp = {}
    for L in legs:
        for lab, v in A.budget(d, L["leg"]):
            comp.setdefault(lab, []).append(abs(v))
    names = {"por paso del motor (25 µm)": "paso", "por ciclo eléctrico (100 µm)": "ciclo", "por imán (83 µm)": "iman",
             "por vuelta de motor (5 mm)": "vuelta", "resto (aleatorio + picos), desvío": "resto", "constante": "const"}
    for lab, v in comp.items():
        out[names[lab]] = float(np.mean(v))
    rest = (d["leg"] == -1) & (np.abs(d["vt_mm_s"]) < 1e-6)
    out["reposo"] = float(np.std(d["err_um"][rest])) if rest.sum() > 100 else np.nan
    out["max_out"] = float(np.max(np.abs(d["out_deg"][mv])))
    out["out_pp_ciclo"] = float(np.ptp(pi_pattern(d, meta)))
    return out


def load_iter(out_dir, it):
    z = np.load(os.path.join(out_dir, it["archivo"]), allow_pickle=False)
    d = {k: z[k] for k in z.files if k != "meta"}
    meta = json.loads(str(z["meta"]))
    T.derive(d, meta)
    return d, meta


def plot(out_dir, iters, labels):
    k = np.arange(len(iters))
    fig = plt.figure(figsize=(16, 9.5))
    gs = fig.add_gridspec(2, 3, hspace=0.45, wspace=0.25)
    a = fig.add_subplot(gs[0, 0])
    a.plot(k, [it["m"]["rms"] for it in iters], "o-", color=T.DIR_COLOR[+1], label="RMS")
    a.plot(k, [it["m"]["paso"] for it in iters], "s-", color=T.DIR_COLOR[-1], label="componente del paso (25 µm)")
    a.plot(k, [it["m"]["ciclo"] for it in iters], "^-", color="#1baf7a", label="componente del ciclo eléctrico (100 µm)")
    a.set_title("(a) Error en movimiento por iteración", loc="left")
    a.set_ylabel("µm")
    a.set_ylim(0, None)
    a.legend(fontsize=8)
    b = fig.add_subplot(gs[0, 1])
    b.plot(k, [it["m"]["max"] for it in iters], "o-", color=T.DIR_COLOR[+1])
    b.set_ylabel("µm")
    b.set_ylim(0, None)
    b.set_title("(b) Máximo |error| en movimiento", loc="left")
    c = fig.add_subplot(gs[0, 2])
    c.plot(k, [it["m"]["out_pp_ciclo"] for it in iters], "o-", color=T.DIR_COLOR[-1])
    c.set_title("(c) Lo que todavía corrige el PI\n(patrón por ciclo eléctrico, pico a pico)", loc="left", fontsize=10)
    c.set_ylabel("° eléctricos")
    c.set_ylim(0, None)
    for ax in (a, b, c):
        ax.set_xticks(k)
        ax.set_xticklabels(labels, fontsize=8)
        ax.set_xlabel("iteración")
    ph = np.arange(N_TABLE) / N_TABLE * 360
    cols = plt.cm.viridis(np.linspace(0.1, 0.9, len(iters)))
    t = fig.add_subplot(gs[1, 0])
    for i, it in enumerate(iters):
        t.plot(ph, it["table_deg"], color=cols[i], lw=1.5, label=f"iter. {i}")
        t.plot(ph, it["table_dir_deg"], color=cols[i], lw=1.2, ls=(0, (3, 2)))
    t.set_title("(d) Tablas: llena = igual en los dos sentidos,\npunteada = cambia de signo con el sentido",
                loc="left", fontsize=10)
    t.set_xlabel("ángulo eléctrico del campo [°]")
    t.set_ylabel("adelanto extra [° el.]")
    t.legend(fontsize=7, ncol=2)
    e = fig.add_subplot(gs[1, 1:])
    for i, it in enumerate(iters):
        for dd, ls in ((+1, "-"), (-1, (0, (3, 2)))):
            if dd in it["err_dir"]:
                e.plot(ph, it["err_dir"][dd], color=cols[i], lw=1.4, ls=ls,
                       label=labels[i].replace("\n", " ") if dd > 0 else None)
    e.axhline(0, color=T.AXIS, lw=0.8)
    e.set_title("(e) Error sobre un ciclo eléctrico (100 µm = 4 pasos), llena = ida, punteada = vuelta", loc="left")
    e.set_xlabel("ángulo eléctrico del campo [°]")
    e.set_ylabel("µm")
    e.legend(fontsize=8, ncol=3)
    fig.suptitle("Compensación de la retención del motor (anti-cogging): aprendizaje por iteraciones",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.05, right=0.98, top=0.92, bottom=0.07)
    p = os.path.join(out_dir, "cogging_iteraciones.png")
    fig.savefig(p, dpi=105)
    plt.close(fig)
    return p


def analyze(out_dir):
    summ = json.load(open(os.path.join(out_dir, "cogging.json")))
    iters = []
    for it in summ["iteraciones"]:
        d, meta = load_iter(out_dir, it)
        it["m"] = metrics(d, meta)
        it["err_dir"] = folds_by_dir(d, meta, "err_um")
        it.setdefault("table_dir_deg", [0.0] * N_TABLE)
        iters.append(it)
    labels = [it.get("tipo", str(i)) for i, it in enumerate(iters)]
    labels = [f"{i}\n({lab})" for i, lab in enumerate(labels)]
    print(f"{'iter':>4s} {'tipo':>10s} {'RMS':>6s} {'máx':>6s} {'paso':>6s} {'ciclo':>6s} {'imán':>6s} {'vuelta':>6s} "
          f"{'resto':>6s} {'reposo':>7s} {'PI pp':>6s} {'máx sal.':>8s}")
    for i, it in enumerate(iters):
        m = it["m"]
        print(f"{i:4d} {it.get('tipo', ''):>10s} {m['rms']:6.2f} {m['max']:6.2f} {m['paso']:6.2f} {m['ciclo']:6.2f} "
              f"{m['iman']:6.2f} {m['vuelta']:6.2f} {m['resto']:6.2f} {m['reposo']:7.3f} {m['out_pp_ciclo']:6.1f} "
              f"{m['max_out']:8.1f}")
    print("  " + plot(out_dir, iters, labels))
    return iters


# ======================================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--end", type=float, default=50.0)
    ap.add_argument("--feed", type=float, default=2.0)
    ap.add_argument("--iters", type=int, default=4, help="iteraciones de aprendizaje")
    ap.add_argument("--gain", type=float, default=0.8, help="ganancia del paso de la secante")
    ap.add_argument("--dwell", type=float, default=3.0, help="reposo en cada punta [s]")
    ap.add_argument("--start-from-flash", action="store_true", help="arrancar desde las tablas guardadas (M59 L)")
    ap.add_argument("--continue-from", nargs="+", help="seguir desde corridas anteriores, en orden (misma velocidad)")
    ap.add_argument("--save", action="store_true", help="guardar en flash las mejores tablas (M59 W)")
    ap.add_argument("--tag", default="")
    ap.add_argument("--from-dir")
    args = ap.parse_args()
    if args.from_dir:
        analyze(args.from_dir)
        return

    out_dir = os.path.join(ROOT, "mediciones", datetime.date.today().isoformat(),
                           "cogging_" + datetime.datetime.now().strftime("%H%M%S") + (f"_{args.tag}" if args.tag else ""))
    os.makedirs(out_dir, exist_ok=True)
    link = T.Link(args.port, args.baud)
    summ = {"fecha": datetime.datetime.now().isoformat(timespec="seconds"), "end_mm": args.end, "feed_mm_s": args.feed,
            "gain": args.gain, "iteraciones": []}
    try:
        time.sleep(0.5)
        link.command("M120 S0", echo=False)
        link.wait_text(0.3)
        link.buf.clear()
        st = T.parse_status(link.command("M119", echo=False))
        if st.get("homed") != "1" or st.get("calibrated") != "1":
            sys.exit("el eje no esta homeado/calibrado: correr M56 primero")
        if args.start_from_flash:
            m59(link, "L")
        ts, ta, field_at_zero = read_tables(link)
        summ["field_at_zero"] = field_at_zero
        summ["m58"] = link.command("M58", echo=False)[0]
        print(f"campo en cero: {field_at_zero:.4f} rad; {summ['m58']}")
        link.command("G90", echo=False)
        link.command("M17", echo=False)
        link.command("M120 S1 P2", echo=False)
        T.run_move(link, 0.0, args.feed, 0.5, "aproximacion")

        history, G_mem, pi_last = [], {}, np.zeros(N_TABLE)
        prev_runs = []
        for cdir in args.continue_from or []:
            prev = json.load(open(os.path.join(cdir, "cogging.json")))
            prev_runs += [(cdir, it) for it in prev["iteraciones"] if it["ok"]]
        if prev_runs:
            for cdir, it in prev_runs:
                dd, mm = load_iter(cdir, it)
                mm["field_at_zero"] = field_at_zero     # el indice es el angulo absoluto del campo
                e = folds_by_dir(dd, mm, "err_um")
                if set(e) != {+1, -1}:
                    continue
                history.append((np.array(it["table_deg"]), np.array(it.get("table_dir_deg", [0.0] * N_TABLE)), e))
                pi_last = pi_pattern(dd, mm)
                if len(history) >= 2:
                    next_tables(history, args.gain, pi_last, G_mem)    # actualiza G_mem con cada par de pasadas
            summ["continua_de"] = args.continue_from
            print(f"continuando desde {args.continue_from}: {len(history)} pasadas previas, "
                  f"{len(G_mem)} respuestas estimadas")
            ts, ta = next_tables(history, args.gain, pi_last, G_mem)
        elif not args.start_from_flash:
            ts, ta = np.zeros(N_TABLE), np.zeros(N_TABLE)

        n_pass = args.iters + (0 if args.continue_from else 1)
        for k in range(n_pass):
            tipo = "secante" if (args.continue_from or k >= 2) else ("prueba" if k == 1 else "referencia")
            enabled = upload(link, ts, ta, args.feed)
            print(f"\n=== iteracion {k} ({tipo}): anti-cogging {'activo' if enabled else 'apagado'}, "
                  f"max {np.max(np.abs(ts)):.2f}° + por sentido {np.max(np.abs(ta)):.2f}°")
            rows, legs_idx, legs = run_pass(link, args, k)
            d = T.rows_to_arrays(rows, legs_idx)
            meta = {"fecha": summ["fecha"], "end_mm": args.end, "cycles": 1, "feed_mm_s": args.feed, "decimation": 2,
                    "legs": legs, "ok": all(L["ok"] for L in legs), "field_at_zero": field_at_zero,
                    "table_deg": ts.tolist(), "table_dir_deg": ta.tolist(), "iter": k}
            fname = f"iter_{k}.npz"
            np.savez_compressed(os.path.join(out_dir, fname), meta=json.dumps(meta), **d)
            T.derive(d, meta)
            m = metrics(d, meta)
            pi_last = pi_pattern(d, meta)
            print(f"   RMS {m['rms']:.2f} µm, máx {m['max']:.2f} µm, paso {m['paso']:.2f} µm, ciclo {m['ciclo']:.2f} µm, "
                  f"reposo {m['reposo']:.3f} µm, PI pp por ciclo {np.ptp(pi_last):.1f}°")
            summ["iteraciones"].append({"iter": k, "tipo": tipo, "archivo": fname, "table_deg": ts.tolist(),
                                        "table_dir_deg": ta.tolist(), "rms": m["rms"], "ok": meta["ok"]})
            json.dump(summ, open(os.path.join(out_dir, "cogging.json"), "w"), indent=1)
            if not meta["ok"]:
                print("   tramo fallido: se corta el aprendizaje")
                break
            e = folds_by_dir(d, meta, "err_um")
            history.append((ts.copy(), ta.copy(), e))
            if k < n_pass - 1:
                ts, ta = next_tables(history, args.gain, pi_last, G_mem)

        ok_iters = [it for it in summ["iteraciones"] if it["ok"]]
        best = min(ok_iters, key=lambda it: it["rms"])
        print(f"\nmejor iteracion: {best['iter']} (RMS {best['rms']:.2f} µm) -> queda cargada")
        summ["mejor"] = best["iter"]
        upload(link, np.array(best["table_deg"]), np.array(best["table_dir_deg"]), args.feed)
        if args.save:
            m59(link, "W")
            print("tablas guardadas en flash (M59 W)")
        json.dump(summ, open(os.path.join(out_dir, "cogging.json"), "w"), indent=1)
    finally:
        try:
            link.command("M120 S0", echo=False)
        finally:
            link.close()
    analyze(out_dir)


if __name__ == "__main__":
    main()
