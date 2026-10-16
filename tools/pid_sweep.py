#!/usr/bin/env python3
"""Barrido de configuraciones del PID en lazo cerrado, con comparacion del error en todo el eje.

Para cada configuracion: cambia las ganancias en caliente con M55, rearma el lazo (M17),
verifica que quede estable en reposo, recorre el carril 0 -> X_fin -> 0 (las veces pedidas)
con telemetria a 1 kHz, y pasa a la siguiente. Al final vuelve a dejar las ganancias de base.
No hace falta recompilar: M55 cambia las ganancias del firmware que esta cargado.

La base son los valores de src/hw_config.h (POS_KP, POS_KI, VEL_KP, VEL_KI, VEL_LOWPASS_TC).
El barrido por defecto cambia UN parametro por vez alrededor de la base, para poder atribuir
cada cambio en el error a ese parametro (ver docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md).

Uso:
    python3 tools/pid_sweep.py                          # barrido por defecto, todo el eje (~4,5 min por config)
    python3 tools/pid_sweep.py --end 50                 # mas corto: 0 -> 50 -> 0 mm (~1 min por config)
    python3 tools/pid_sweep.py --list                   # mostrar las configuraciones sin mover nada
    python3 tools/pid_sweep.py --config "kp120:pos_kp=120" --config "ki0:pos_ki=0,pos_kp=80"
    python3 tools/pid_sweep.py --from-dir mediciones/AAAA-MM-DD/barrido_pid_HHMMSS   # re-analizar

Parametros de --config (los que no se dan quedan como en la base):
    pos_kp, pos_ki   lazo de posicion   [1/s], [1/s^2]
    vel_kp, vel_ki   lazo de velocidad  [rad/(rad/s)], [rad/(rad/s)/s]
    vel_tc           filtro de la velocidad medida [s]

Seguridad: con cada configuracion nueva se mira 1,5 s en reposo; si el error oscila mas de
--rest-limit-um pico a pico o la salida toca el limite, se descarta y se vuelve a la base. En
movimiento, si |error| pasa --abort-um, se vuelve a la base y se lleva el carro al 0.

Requiere el eje homeado y calibrado (M56 recomendado, ver docs/CAMBIOS_2026-10-16.md). Cerrar
cualquier otro programa que tenga el puerto serie abierto.

Resultados en mediciones/AAAA-MM-DD/barrido_pid_HHMMSS/:
    cfg_<nombre>.npz          datos de cada configuracion (mismo formato que closed_loop_test.py;
                              se pueden graficar con closed_loop_test.py --from-file)
    resumen.csv / resumen.txt metricas por configuracion
    comparacion_metricas.png  cada metrica del error, una fila por configuracion
    comparacion_eje.png       error a lo largo de todo el eje, un panel por configuracion
    comparacion_plegado.png   error plegado sobre el ciclo electrico (100 um = 4 pasos) y sobre 2 imanes
"""
import argparse
import csv
import datetime
import glob
import json
import os
import re
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import closed_loop_test as T  # noqa: E402  (enlace serie, telemetria y estilo)
import analisis_errores as A  # noqa: E402  (carga, derivadas y presupuesto del error)
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS = ["pos_kp", "pos_ki", "vel_kp", "vel_ki", "vel_tc"]
HW_NAMES = {"pos_kp": "POS_KP", "pos_ki": "POS_KI", "vel_kp": "VEL_KP", "vel_ki": "VEL_KI",
            "vel_tc": "VEL_LOWPASS_TC"}
DEFAULT_PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00"


# ======================================================================================
# Configuraciones
# ======================================================================================

def read_hw_config(path=os.path.join(ROOT, "src", "hw_config.h")):
    """Ganancias de base tal como estan en hw_config.h (lo que el firmware usa al arrancar)."""
    text = open(path).read()
    base = {}
    for key, name in HW_NAMES.items():
        m = re.search(rf"constexpr\s+float\s+{name}\s*=\s*([-0-9.eE+]+)f?\s*;", text)
        if not m:
            raise SystemExit(f"no encontre {name} en {path}")
        base[key] = float(m.group(1))
    return base


def default_grid(base):
    """Un parametro por vez alrededor de la base."""
    def cfg(name, **changes):
        p = dict(base)
        p.update(changes)
        return {"name": name, "params": p}
    b = base
    return [
        cfg("base"),
        cfg("pos_kp_x0.5", pos_kp=b["pos_kp"] * 0.5),
        cfg("pos_kp_x2", pos_kp=b["pos_kp"] * 2),
        cfg("pos_ki_0", pos_ki=0.0),
        cfg("pos_ki_x0.5", pos_ki=b["pos_ki"] * 0.5),
        cfg("pos_ki_x2", pos_ki=b["pos_ki"] * 2),
        cfg("vel_kp_x0.5", vel_kp=b["vel_kp"] * 0.5),
        cfg("vel_kp_x1.5", vel_kp=b["vel_kp"] * 1.5),
        cfg("vel_ki_x0.5", vel_ki=b["vel_ki"] * 0.5),
        cfg("vel_ki_x2", vel_ki=b["vel_ki"] * 2),
        cfg("vel_tc_x0.5", vel_tc=b["vel_tc"] * 0.5),
        cfg("vel_tc_x2", vel_tc=b["vel_tc"] * 2),
        cfg("base_final"),            # repetir la base al final: muestra la deriva durante el barrido
    ]


def parse_config(text, base):
    """'nombre:pos_kp=120,vel_ki=60' -> configuracion (lo que no se da queda como en la base)."""
    name, _, rest = text.partition(":")
    p = dict(base)
    for item in filter(None, rest.split(",")):
        k, _, v = item.partition("=")
        k = k.strip()
        if k not in PARAMS:
            raise SystemExit(f"parametro desconocido '{k}' (validos: {', '.join(PARAMS)})")
        p[k] = float(v)
    return {"name": name.strip(), "params": p}


def m55(p):
    return f"M55 A{p['pos_kp']:g} B{p['pos_ki']:g} C{p['vel_kp']:g} D{p['vel_ki']:g} F{p['vel_tc']:g}"


def describe(cfg, base):
    diff = [f"{k}={cfg['params'][k]:g}" for k in PARAMS if cfg["params"][k] != base[k]]
    return ", ".join(diff) if diff else "base"


# ======================================================================================
# Captura
# ======================================================================================

def wait_rows(link, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        link.poll()


def rest_check(link, seconds):
    """Error pico a pico [um] y maximo |salida| [grados] en reposo durante 'seconds'."""
    n0 = len(link.rows)
    wait_rows(link, seconds)
    r = np.array(link.rows[n0:], dtype=float)
    if len(r) < 10:
        raise RuntimeError("no llega telemetria en el control de reposo")
    err = 1000 * (r[:, 1] - r[:, 2])
    return float(np.ptp(err)), float(np.max(np.abs(np.degrees(r[:, 4])))), bool(np.any(r[:, 7].astype(int) & 2))


def back_to_base(link, base, args, why):
    """Vuelve a las ganancias de base, rearma el lazo si hizo falta y lleva el carro al 0."""
    print(f"   -> vuelvo a la base ({why})")
    link.command(m55(base), echo=False)
    link.command("M17", echo=False)
    wait_rows(link, 0.5)
    link.current_leg = -2
    T.run_move(link, 0.0, args.feed, args.settle, "retorno al 0 con la base")


def save_config(out_dir, cfg, base, rows, legs, meta_common, legs_meta, status):
    d = T.rows_to_arrays(rows, legs)
    meta = dict(meta_common)
    meta.update({"legs": legs_meta, "ok": status == "ok", "estado": status, "config": cfg["name"],
                 "pid": cfg["params"], "pid_base": base, "cambio": describe(cfg, base)})
    path = os.path.join(out_dir, f"cfg_{cfg['name']}.npz")
    np.savez_compressed(path, meta=json.dumps(meta), **d)
    return path


def sweep(args, configs, base, out_dir):
    link = T.Link(args.port, args.baud)
    meta_common = {"fecha": datetime.datetime.now().isoformat(timespec="seconds"), "end_mm": args.end,
                   "cycles": args.cycles, "feed_mm_s": args.feed, "decimation": args.decimation}
    log = {"configs": [], "base": base}
    try:
        time.sleep(0.5)
        link.command("M120 S0")
        link.wait_text(0.3)
        link.buf.clear()
        st = T.parse_status(link.command("M119"))
        if st.get("homed") != "1" or st.get("calibrated") != "1":
            raise SystemExit("el eje no esta homeado/calibrado: correr M56 (o G28) primero")
        meta_common["m58"] = link.command("M58")[0]
        link.command("G90")
        link.command(m55(base))
        link.command("M17")
        link.command(f"M120 S1 P{args.decimation}")
        wait_rows(link, 0.5)
        link.current_leg = -2
        if abs(link.rows[-1][2]) > 0.01:
            T.run_move(link, 0.0, args.feed, args.settle, "aproximacion")

        for i, cfg in enumerate(configs):
            p = cfg["params"]
            print(f"\n=== [{i + 1}/{len(configs)}] {cfg['name']}: {describe(cfg, base)}")
            link.rows.clear()
            link.leg.clear()
            link.current_leg = -1
            link.command(m55(p), echo=False)
            link.command("M17", echo=False)
            wait_rows(link, 0.3)          # dejar pasar la rampa de corriente del rearme
            pp, out_max, fault = rest_check(link, 1.5)
            print(f"   reposo con la config nueva: error {pp:.2f} um pico a pico, |salida| max {out_max:.0f}°")
            status = "ok"
            legs_meta = []
            if fault or pp > args.rest_limit_um or out_max > T.OUT_LIMIT_DEG - 0.5:
                status = "inestable en reposo"
            else:
                dests = []
                for c in range(args.cycles):
                    dests += [(args.end, +1, c), (0.0, -1, c)]
                for k, (dest, dsign, c) in enumerate(dests):
                    link.current_leg = k
                    name = f"{'ida' if dsign > 0 else 'vuelta'} {c + 1}"
                    t0 = time.time()
                    ok = T.run_move(link, dest, args.feed, args.settle, f"{cfg['name']} {name}",
                                    abort_err_um=args.abort_um)
                    legs_meta.append({"leg": k, "name": name, "dir": dsign, "cycle": c, "dest": dest, "ok": ok,
                                      "dur_s": round(time.time() - t0, 2)})
                    link.current_leg = -1
                    if not ok:
                        status = "corte por falla" if (int(link.rows[-1][7]) & 2) else "abortada (error grande)"
                        break
                    wait_rows(link, args.dwell)
            rows, legs = list(link.rows), list(link.leg)
            if status != "ok":
                print(f"   CONFIG DESCARTADA: {status}")
                back_to_base(link, base, args, status)
            path = save_config(out_dir, cfg, base, rows, legs, meta_common, legs_meta, status)
            log["configs"].append({"name": cfg["name"], "params": p, "estado": status, "archivo": os.path.basename(path)})
            with open(os.path.join(out_dir, "barrido.json"), "w") as fh:
                json.dump(log, fh, indent=1)
    except KeyboardInterrupt:
        print("\ninterrumpido: vuelvo a la base")
        try:
            link.command(m55(base), echo=False)
            link.command("M17", echo=False)
        except Exception:
            pass
    finally:
        try:
            link.command("M120 S0")
            link.wait_text(0.3)
            link.command(m55(base))
        finally:
            link.close()
    return log


# ======================================================================================
# Analisis
# ======================================================================================

METRICS = [("rms", "RMS del error\n[µm]"), ("max_err", "máximo |error|\nen movimiento [µm]"),
           ("const", "atraso constante\n[µm]"), ("iman", "por imán\n(83 µm) [µm]"),
           ("paso", "por paso\n(25 µm) [µm]"), ("ciclo", "por ciclo eléctrico\n(100 µm) [µm]"),
           ("vuelta", "por vuelta\n(5 mm) [µm]"), ("resto", "resto\n(desvío) [µm]"),
           ("reposo", "reposo\n(desvío) [µm]"), ("max_out", "máximo |salida|\n[° el.]")]


def metrics(d, meta):
    """Metricas de una configuracion, promediando los tramos completos (ida y vuelta)."""
    legs = [L for L in meta["legs"] if L["ok"]]
    out = {k: np.nan for k, _ in METRICS}
    rest = (d["leg"] == -1) & (np.abs(d["vt_mm_s"]) < 1e-6)
    if rest.sum() > 50:
        out["reposo"] = float(np.std(d["err_um"][rest]))
    if not legs:
        return out
    moving = np.isin(d["leg"], [L["leg"] for L in legs])
    cruise = moving & d["cruise"]
    if cruise.sum() < 1000:
        return out
    out["rms"] = float(np.sqrt(np.mean(d["err_um"][cruise] ** 2)))
    out["max_err"] = float(np.max(np.abs(d["err_um"][moving])))
    out["max_out"] = float(np.max(np.abs(d["out_deg"][moving])))
    comp = {"const": [], "iman": [], "paso": [], "ciclo": [], "vuelta": [], "resto": []}
    keys = {"constante": "const", "por imán (83 µm)": "iman", "por paso del motor (25 µm)": "paso",
            "por ciclo eléctrico (100 µm)": "ciclo", "por vuelta de motor (5 mm)": "vuelta",
            "resto (aleatorio + picos), desvío": "resto"}
    for L in legs:
        if ((d["leg"] == L["leg"]) & d["cruise"]).sum() < 1000:
            continue
        for lab, v in A.budget(d, L["leg"]):
            comp[keys[lab]].append(abs(v))
    for k, v in comp.items():
        out[k] = float(np.mean(v)) if v else np.nan
    return out


def load_dir(out_dir):
    runs = []
    order = []
    log_path = os.path.join(out_dir, "barrido.json")
    if os.path.exists(log_path):
        order = [c["archivo"] for c in json.load(open(log_path))["configs"]]
    files = order or sorted(os.path.basename(f) for f in glob.glob(os.path.join(out_dir, "cfg_*.npz")))
    for f in files:
        d, meta = A.load(os.path.join(out_dir, f))
        runs.append((meta, d, metrics(d, meta)))
    return runs


def write_summary(runs, out_dir):
    cols = ["config", "cambio", "estado"] + PARAMS + [k for k, _ in METRICS]
    with open(os.path.join(out_dir, "resumen.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        for meta, d, m in runs:
            w.writerow([meta["config"], meta["cambio"], meta["estado"]] + [meta["pid"][k] for k in PARAMS] +
                       [f"{m[k]:.3f}" for k, _ in METRICS])
    lines = [f"Barrido de PID {runs[0][0]['fecha']}: 0 -> {runs[0][0]['end_mm']:g} mm, {runs[0][0]['cycles']} ciclo(s), "
             f"{runs[0][0]['feed_mm_s']:g} mm/s, {runs[0][0].get('m58', '')}",
             "base: " + ", ".join(f"{k}={runs[0][0]['pid_base'][k]:g}" for k in PARAMS), "",
             f"{'config':14s} {'cambio':26s} {'estado':22s} {'rms':>6s} {'max':>6s} {'const':>6s} {'imán':>5s} "
             f"{'paso':>5s} {'ciclo':>5s} {'vuelta':>6s} {'resto':>5s} {'reposo':>6s} {'max out':>7s}",
             f"{'':14s} {'':26s} {'':22s} {'[µm]':>6s} {'[µm]':>6s} {'[µm]':>6s} {'[µm]':>5s} {'[µm]':>5s} "
             f"{'[µm]':>5s} {'[µm]':>6s} {'[µm]':>5s} {'[µm]':>6s} {'[°]':>7s}"]
    for meta, d, m in runs:
        lines.append(f"{meta['config']:14s} {meta['cambio'][:26]:26s} {meta['estado'][:22]:22s} {m['rms']:6.2f} "
                     f"{m['max_err']:6.1f} {m['const']:6.2f} {m['iman']:5.2f} {m['paso']:5.2f} {m['ciclo']:5.2f} "
                     f"{m['vuelta']:6.2f} {m['resto']:5.2f} {m['reposo']:6.2f} {m['max_out']:7.1f}")
    lines.append("\ncomponentes: amplitud (mitad del pico a pico de la forma plegada), promedio de los tramos; "
                 "rms, resto y reposo: desvío estándar")
    text = "\n".join(lines)
    print("\n" + text)
    with open(os.path.join(out_dir, "resumen.txt"), "w") as fh:
        fh.write(text + "\n")


def fig_metricas(runs, out_dir):
    n = len(runs)
    ncols = 5
    nrows = int(np.ceil(len(METRICS) / ncols))
    fig, axs = plt.subplots(nrows, ncols, figsize=(17, 0.42 * n * nrows + 2.2), sharey=True,
                            gridspec_kw=dict(wspace=0.12, hspace=0.35))
    axs = np.atleast_2d(axs)
    y = np.arange(n)
    base_idx = [i for i, (meta, _, _) in enumerate(runs) if meta["cambio"] == "base"]
    for j, (key, label) in enumerate(METRICS):
        a = axs[j // ncols, j % ncols]
        vals = np.array([m[key] for _, _, m in runs])
        ok = np.array([meta["estado"] == "ok" for meta, _, _ in runs])
        for i in base_idx:
            if np.isfinite(vals[i]):
                a.axvline(vals[i], color=T.MUTED, lw=0.8, ls=(0, (3, 2)))
        a.hlines(y, 0, np.nan_to_num(vals), color=T.GRID, lw=1)
        a.plot(vals[ok], y[ok], "o", color=T.DIR_COLOR[+1], ms=7)
        a.plot(np.nan_to_num(vals[~ok]), y[~ok], "x", color=T.CRITICAL, ms=8)
        for yi, v in zip(y, vals):
            if np.isfinite(v):
                a.text(v, yi - 0.28, f" {v:.2f}" if v < 10 else f" {v:.1f}", fontsize=7, color=T.INK2, va="center")
        if key == "max_out":
            a.axvline(T.OUT_LIMIT_DEG, color=T.CRITICAL, lw=1, ls=(0, (2, 2)))
        a.set_title(label, fontsize=9, loc="left")
        a.set_xlim(0, None)
        a.grid(axis="y", visible=False)
    for j in range(len(METRICS), nrows * ncols):
        axs[j // ncols, j % ncols].set_visible(False)
    for r in range(nrows):
        axs[r, 0].set_yticks(y)
        axs[r, 0].set_yticklabels([f"{meta['config']}\n({meta['cambio']})" if meta["cambio"] != "base"
                                   else meta["config"] for meta, _, _ in runs], fontsize=8)
        axs[r, 0].invert_yaxis()
    fig.suptitle("Barrido de PID: cada métrica del error por configuración (línea punteada = base, × = descartada)",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.13, right=0.98, top=0.9, bottom=0.05)
    A.save(fig, out_dir, "comparacion_metricas.png")


def fig_eje(runs, out_dir):
    end = runs[0][0]["end_mm"]
    edges = np.arange(0, end + T.ENC_PERIOD_MM, T.ENC_PERIOD_MM * 3)   # cada 3 periodos (0,5 mm)
    xc = 0.5 * (edges[1:] + edges[:-1])
    lim = 0.0
    for meta, d, m in runs:
        mv = d["leg"] >= 0
        if mv.any() and meta["estado"] == "ok":
            lim = max(lim, np.nanpercentile(np.abs(d["err_um"][mv]), 99.9))
    lim = max(lim * 1.1, 2.0)
    n = len(runs)
    fig, axs = plt.subplots(n, 1, figsize=(16, 1.55 * n + 1.2), sharex=True, sharey=True,
                            gridspec_kw=dict(hspace=0.35))
    axs = np.atleast_1d(axs)
    for a, (meta, d, m) in zip(axs, runs):
        for L in meta["legs"]:
            if L["cycle"] != 0:
                continue
            mk = d["leg"] == L["leg"]
            if not mk.any():
                continue
            mean, lo, hi = T.binned(d["pos_mm"][mk], d["err_um"][mk], edges)
            color = T.DIR_COLOR[L["dir"]]
            a.fill_between(xc, lo, hi, color=color, alpha=0.15, lw=0)
            a.plot(xc, mean, color=color, lw=1)
        a.axhline(0, color=T.AXIS, lw=0.8)
        state = "" if meta["estado"] == "ok" else f"  —  {meta['estado'].upper()}"
        a.set_title(f"{meta['config']} ({meta['cambio']}):  RMS {m['rms']:.2f} µm, máx {m['max_err']:.1f} µm{state}",
                    fontsize=9, loc="left", color=T.CRITICAL if state else T.INK)
        a.set_ylim(-lim, lim)
        a.set_ylabel("µm", fontsize=8)
    axs[-1].set_xlabel("posición en el carril [mm]   (línea = media cada 0,5 mm, banda = mín–máx; "
                       "azul = ida, naranja = vuelta, primer ciclo)")
    axs[-1].set_xlim(0, end)
    fig.suptitle("Error de seguimiento a lo largo del eje para cada configuración del PID (misma escala en todos)",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.05, right=0.98, top=1 - 0.6 / (1.55 * n + 1.2), bottom=0.6 / (1.55 * n + 1.2))
    A.save(fig, out_dir, "comparacion_eje.png")


def fig_plegado(runs, out_dir):
    n = len(runs)
    ncols = min(4, n)
    nrows = int(np.ceil(n / ncols))
    fig, axs = plt.subplots(nrows * 2, ncols, figsize=(4.2 * ncols, 2.3 * nrows * 2 + 1), sharey="row",
                            gridspec_kw=dict(hspace=0.55, wspace=0.08), squeeze=False)
    e100 = np.linspace(0, 100, 41)
    e360 = np.linspace(0, 360, 61)
    for i, (meta, d, m) in enumerate(runs):
        r, c = divmod(i, ncols)
        a1, a2 = axs[2 * r, c], axs[2 * r + 1, c]
        for L in meta["legs"]:
            if L["cycle"] != 0 or not L["ok"]:
                continue
            mk = (d["leg"] == L["leg"]) & d["cruise"]
            if mk.sum() < 1000:
                continue
            y = d["err_um"][mk] - d["err_um"][mk].mean()
            mean, _, _ = T.binned((d["pos_mm"][mk] * 1000) % 100, y, e100)
            a1.plot(0.5 * (e100[1:] + e100[:-1]), mean, color=T.DIR_COLOR[L["dir"]], lw=1.5)
            mean, _, _ = T.binned(d["enc_deg"][mk], y, e360)
            a2.plot(0.5 * (e360[1:] + e360[:-1]), mean, color=T.DIR_COLOR[L["dir"]], lw=1.5)
        for x in (25, 50, 75):
            a1.axvline(x, color=T.GRID, lw=0.8)
        a1.set_title(f"{meta['config']}" + ("" if meta["estado"] == "ok" else f" ({meta['estado']})"), fontsize=9,
                     loc="left", color=T.INK if meta["estado"] == "ok" else T.CRITICAL)
        a1.set_xlim(0, 100)
        a2.set_xlim(0, 360)
        a2.set_xticks([0, 180, 360])
        if c == 0:
            a1.set_ylabel("ciclo eléctrico\n[µm]", fontsize=8)
            a2.set_ylabel("2 imanes\n[µm]", fontsize=8)
        a1.set_xlabel("posición en el ciclo [µm]", fontsize=8)
        a2.set_xlabel("ángulo del encoder [°]", fontsize=8)
    for j in range(n, nrows * ncols):
        r, c = divmod(j, ncols)
        axs[2 * r, c].set_visible(False)
        axs[2 * r + 1, c].set_visible(False)
    fig.suptitle("Error plegado (sin la media) sobre el ciclo eléctrico (líneas = cada paso de 25 µm) y sobre 2 imanes; "
                 "azul = ida, naranja = vuelta", x=0.01, ha="left", fontsize=11, fontweight="bold")
    fig.subplots_adjust(left=0.07, right=0.98, top=1 - 0.7 / (2.3 * nrows * 2 + 1), bottom=0.05)
    A.save(fig, out_dir, "comparacion_plegado.png")


def analyze(out_dir):
    runs = load_dir(out_dir)
    if not runs:
        raise SystemExit(f"no hay cfg_*.npz en {out_dir}")
    write_summary(runs, out_dir)
    print("\ngraficos:")
    fig_metricas(runs, out_dir)
    fig_eje(runs, out_dir)
    fig_plegado(runs, out_dir)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--config", action="append", default=[],
                    help="'nombre:param=valor,...' (se puede repetir; reemplaza el barrido por defecto)")
    ap.add_argument("--no-base", action="store_true", help="con --config, no agregar la base al principio y al final")
    ap.add_argument("--list", action="store_true", help="mostrar las configuraciones y salir")
    ap.add_argument("--end", type=float, default=255.0, help="punta lejana [mm]")
    ap.add_argument("--cycles", type=int, default=1, help="idas y vueltas por configuracion")
    ap.add_argument("--feed", type=float, default=2.0, help="velocidad [mm/s]")
    ap.add_argument("--decimation", type=int, default=2, help="M120 P: 2 = 1 kHz")
    ap.add_argument("--settle", type=float, default=0.5)
    ap.add_argument("--dwell", type=float, default=1.0, help="reposo en cada punta [s]")
    ap.add_argument("--rest-limit-um", type=float, default=10.0,
                    help="descartar la config si en reposo el error oscila mas que esto (pico a pico)")
    ap.add_argument("--abort-um", type=float, default=150.0,
                    help="en movimiento, abortar la config si |error| pasa esto")
    ap.add_argument("--from-dir", help="re-analizar una carpeta de barrido ya capturada")
    args = ap.parse_args()

    if args.from_dir:
        analyze(args.from_dir)
        return

    base = read_hw_config()
    if args.config:
        configs = [parse_config(c, base) for c in args.config]
        if not args.no_base:
            configs = [{"name": "base", "params": dict(base)}] + configs + [{"name": "base_final", "params": dict(base)}]
    else:
        configs = default_grid(base)
    names = [c["name"] for c in configs]
    if len(set(names)) != len(names):
        raise SystemExit("hay nombres de configuracion repetidos")

    per_cfg_s = args.cycles * 2 * (args.end / args.feed + 2.5) + 3
    print("base (src/hw_config.h): " + ", ".join(f"{k}={base[k]:g}" for k in PARAMS))
    for c in configs:
        print(f"  {c['name']:14s} {describe(c, base):30s} {m55(c['params'])}")
    print(f"{len(configs)} configuraciones, ~{per_cfg_s / 60:.1f} min cada una, ~{len(configs) * per_cfg_s / 60:.0f} min en total")
    if args.list:
        return

    out_dir = os.path.join(ROOT, "mediciones", datetime.date.today().isoformat(),
                           "barrido_pid_" + datetime.datetime.now().strftime("%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print(f"resultados en {out_dir}")
    sweep(args, configs, base, out_dir)
    analyze(out_dir)


if __name__ == "__main__":
    main()
