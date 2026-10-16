#!/usr/bin/env python3
"""Barrido de la ganancia integral del lazo de posicion (POS_KI): error en reposo.

Para cada valor de POS_KI:
  1. lo escribe en src/hw_config.h, compila y carga el firmware;
  2. calibra (M56) y habilita el lazo (M17);
  3. va a cada posicion pedida (por defecto X20 y X100) y, una vez asentado, adquiere el
     error (objetivo - posicion) durante --acq segundos a 1 kHz; lo guarda en un CSV;
  4. vuelve al 0 antes de pasar al siguiente valor.
Al final deja src/hw_config.h como estaba y vuelve a cargar ese firmware.

Graficos (uno por posicion): un subplot por valor de POS_KI con el error en el tiempo; en la
leyenda, la media y la varianza. Ademas, un resumen de media, desvio y pico a pico contra POS_KI.

Uso:
    python3 tools/ki_sweep.py                                   # 500, 700, ..., 2900, 3000
    python3 tools/ki_sweep.py --values 500 1000 2000 --positions 20 100 200
    python3 tools/ki_sweep.py --param VEL_KI --values 30 60 90 120
    python3 tools/ki_sweep.py --from-dir mediciones/AAAA-MM-DD/barrido_ki_HHMMSS   # solo graficar

Cerrar antes el monitor serie. Cada valor tarda ~5 min (carga + calibracion + movimientos).
"""
import argparse
import csv
import datetime
import json
import os
import re
import subprocess
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import closed_loop_test as T  # noqa: E402  (enlace serie, telemetria, movimientos y estilo)
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HW_CONFIG = os.path.join(ROOT, "src", "hw_config.h")
PIO = os.path.expanduser("~/.platformio/penv/bin/pio")
DEFAULT_PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00"
CSV_COLS = ["t_s", "target_mm", "pos_mm", "err_um", "vel_rad_s", "out_deg", "chip_raw", "status", "flags"]


# ======================================================================================
# Firmware
# ======================================================================================

def set_param(text, name, value):
    pat = rf"(constexpr\s+float\s+{name}\s*=\s*)([-0-9.eE+]+)(f?\s*;)"
    if not re.search(pat, text):
        raise SystemExit(f"no encontre {name} en {HW_CONFIG}")
    return re.sub(pat, lambda m: f"{m.group(1)}{value:.1f}{m.group(3)}", text, count=1)


def build_and_upload(port, log):
    for attempt in range(3):
        r = subprocess.run([PIO, "run", "-t", "upload"], cwd=ROOT, capture_output=True, text=True)
        log.write(r.stdout[-3000:] + r.stderr[-2000:] + "\n")
        if r.returncode == 0 and "SUCCESS" in r.stdout:
            break
        print(f"   carga fallida (intento {attempt + 1}), reintento...")
        time.sleep(3)
    else:
        raise RuntimeError("no se pudo compilar/cargar el firmware (ver carga.log)")
    # la placa se reinicia: esperar a que vuelva el puerto
    t0 = time.time()
    time.sleep(2)
    while not os.path.exists(port):
        if time.time() - t0 > 30:
            raise RuntimeError("la placa no volvio a aparecer despues de la carga")
        time.sleep(0.5)
    time.sleep(2)


def calibrate(link):
    lines = link.command("M56", timeout=600)
    if not lines or not lines[-1].startswith("ok"):
        raise RuntimeError("fallo la calibracion: " + " | ".join(lines[-3:]))
    info = {}
    for line in lines:
        m = re.search(r"rms_error = ([0-9.]+) deg", line)
        if m:
            info["cal_rms_deg"] = float(m.group(1))
        m = re.search(r"costura de la vuelta: ([-0-9.]+) deg", line)
        if m:
            info["costura_deg"] = float(m.group(1))
        m = re.search(r"Conmutacion: desfase ([-0-9.]+) deg", line)
        if m:
            info["conmutacion_deg"] = float(m.group(1))
    return info


# ======================================================================================
# Adquisicion
# ======================================================================================

def acquire(link, seconds):
    n0 = len(link.rows)
    t0 = time.time()
    while time.time() - t0 < seconds:
        link.poll()
    return list(link.rows[n0:])


def save_csv(path, rows):
    a = np.array(rows, dtype=float)
    t = a[:, 0].astype(np.int64)
    t_s = np.concatenate([[0], np.cumsum(np.diff(t) % (1 << 32))]) * 1e-6
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_COLS)
        for i in range(len(a)):
            _, tgt, pos, vel, out, raw, status, flags, _ = a[i]
            w.writerow([f"{t_s[i]:.6f}", f"{tgt:.6f}", f"{pos:.6f}", f"{1000 * (tgt - pos):.4f}", f"{vel:.5f}",
                        f"{np.degrees(out):.3f}", int(raw), int(status), int(flags)])


def run_value(args, value, out_dir, log):
    tag = f"{args.param.lower()}_{value:g}"
    print(f"\n=== {args.param} = {value:g}")
    text = open(HW_CONFIG).read()
    open(HW_CONFIG, "w").write(set_param(text, args.param, value))
    print("   compilando y cargando...")
    build_and_upload(args.port, log)
    link = T.Link(args.port, args.baud)
    rec = {"param": args.param, "value": value, "archivos": {}, "ok": True}
    try:
        time.sleep(0.5)
        link.command("M120 S0", echo=False)
        link.wait_text(0.3)
        link.buf.clear()
        rec["m58"] = link.command("M58", echo=False)[0]
        print("   calibrando (M56)...")
        rec.update(calibrate(link))
        print(f"   calibracion: rms {rec.get('cal_rms_deg', float('nan')):.3f}°, "
              f"conmutacion {rec.get('conmutacion_deg', float('nan')):+.1f}°")
        link.command("G90", echo=False)
        link.command("M17", echo=False)
        link.command(f"M120 S1 P{args.decimation}", echo=False)
        acquire(link, 0.5)
        for x in args.positions:
            if not T.run_move(link, x, args.feed, args.settle, f"{tag} G1 X{x:g}", abort_err_um=args.abort_um):
                rec["ok"] = False
                print("   FALLO el movimiento: se pasa al siguiente valor")
                break
            rows = acquire(link, args.acq)
            fname = f"{tag}_x{x:g}.csv"
            save_csv(os.path.join(out_dir, fname), rows)
            e = 1000 * (np.array([r[1] for r in rows]) - np.array([r[2] for r in rows]))
            rec["archivos"][f"{x:g}"] = fname
            print(f"   X{x:g}: {len(rows)} muestras, media {e.mean():+.3f} µm, varianza {e.var():.4f} µm², "
                  f"pico a pico {np.ptp(e):.2f} µm")
        if rec["ok"] or (link.rows and not (int(link.rows[-1][7]) & 2)):
            T.run_move(link, 0.0, args.feed, args.settle, f"{tag} vuelta al 0")
    finally:
        try:
            link.command("M120 S0", echo=False)
        except Exception:
            pass
        link.close()
    return rec


# ======================================================================================
# Graficos
# ======================================================================================

def load_runs(out_dir):
    meta = json.load(open(os.path.join(out_dir, "barrido.json")))
    runs = []
    for rec in meta["valores"]:
        for x, fname in rec["archivos"].items():
            d = np.genfromtxt(os.path.join(out_dir, fname), delimiter=",", names=True)
            runs.append((rec, float(x), d))
    return meta, runs


def fig_position(meta, runs, x, out_dir):
    sel = [(rec, d) for rec, xx, d in runs if xx == x]
    if not sel:
        return None
    n = len(sel)
    ncols = 4 if n > 6 else min(3, n)
    nrows = int(np.ceil(n / ncols))
    lim = max(np.max(np.abs(d["err_um"])) for _, d in sel) * 1.15
    fig, axs = plt.subplots(nrows, ncols, figsize=(4.4 * ncols, 2.6 * nrows + 1.0), sharex=True, sharey=True,
                            squeeze=False, gridspec_kw=dict(hspace=0.4, wspace=0.08))
    for i, (rec, d) in enumerate(sel):
        a = axs[i // ncols, i % ncols]
        e = d["err_um"]
        a.plot(d["t_s"], e, color=T.DIR_COLOR[+1], lw=0.7,
               label=f"media = {e.mean():+.3f} µm\nvarianza = {e.var():.4f} µm²")
        a.axhline(0, color=T.AXIS, lw=0.8)
        a.set_title(f"{meta['param']} = {rec['value']:g}", fontsize=10)
        a.legend(loc="upper right", fontsize=8, handlelength=0, handletextpad=0, frameon=True,
                 facecolor=T.SURFACE, edgecolor=T.GRID)
        a.set_ylim(-lim, lim)
        if i % ncols == 0:
            a.set_ylabel("objetivo − posición [µm]")
        if i // ncols == nrows - 1 or i + ncols >= n:
            a.set_xlabel("tiempo desde que se asentó [s]")
            a.tick_params(labelbottom=True)
    for j in range(n, nrows * ncols):
        axs[j // ncols, j % ncols].set_visible(False)
    fig.suptitle(f"Error en reposo en X = {x:g} mm para cada {meta['param']} "
                 f"({meta['acq_s']:g} s a {2000 / meta['decimation']:.0f} Hz; misma escala en todos)",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.07, right=0.98, top=1 - 0.75 / (2.6 * nrows + 1.0), bottom=0.6 / (2.6 * nrows + 1.0))
    path = os.path.join(out_dir, f"error_reposo_x{x:g}.png")
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


def fig_resumen(meta, runs, out_dir):
    xs = sorted(set(x for _, x, _ in runs))
    colors = ["#2a78d6", "#eb6834", "#1baf7a"]
    fig, axs = plt.subplots(1, 3, figsize=(16, 4.6), gridspec_kw=dict(wspace=0.25))
    for j, x in enumerate(xs):
        sel = sorted([(rec["value"], d["err_um"]) for rec, xx, d in runs if xx == x])
        v = np.array([s[0] for s in sel])
        for a, f, lab in zip(axs, (np.mean, np.std, np.ptp), ("media [µm]", "desvío estándar [µm]", "pico a pico [µm]")):
            a.plot(v, [f(e) for _, e in sel], "o-", color=colors[j % 3], lw=1.5, ms=6, label=f"X = {x:g} mm")
            a.set_title(lab, loc="left")
            a.set_xlabel(meta["param"])
    axs[0].axhline(0, color=T.AXIS, lw=0.8)
    axs[0].legend()
    for a in axs[1:]:
        a.set_ylim(0, None)
    fig.suptitle(f"Error en reposo contra {meta['param']}", x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.05, right=0.98, top=0.83, bottom=0.14)
    path = os.path.join(out_dir, "resumen_vs_ki.png")
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return path


def analyze(out_dir):
    meta, runs = load_runs(out_dir)
    lines = [f"{'valor':>8s} {'X':>5s} {'media [µm]':>11s} {'varianza [µm²]':>15s} {'desvío [µm]':>12s} "
             f"{'pico a pico':>11s} {'cal rms':>8s} {'conmut.':>8s}"]
    for rec, x, d in sorted(runs, key=lambda r: (r[1], r[0]["value"])):
        e = d["err_um"]
        lines.append(f"{rec['value']:8g} {x:5g} {e.mean():+11.3f} {e.var():15.4f} {e.std():12.3f} {np.ptp(e):11.2f} "
                     f"{rec.get('cal_rms_deg', np.nan):8.3f} {rec.get('conmutacion_deg', np.nan):+8.1f}")
    text = "\n".join(lines)
    print("\n" + text)
    open(os.path.join(out_dir, "resumen.txt"), "w").write(text + "\n")
    print("\ngraficos:")
    for x in sorted(set(x for _, x, _ in runs)):
        print("  " + fig_position(meta, runs, x, out_dir))
    print("  " + fig_resumen(meta, runs, out_dir))


# ======================================================================================

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--param", default="POS_KI", help="constante de src/hw_config.h a barrer")
    ap.add_argument("--values", type=float, nargs="+",
                    default=[float(v) for v in np.arange(500, 3000, 200)] + [3000.0])
    ap.add_argument("--positions", type=float, nargs="+", default=[20.0, 100.0])
    ap.add_argument("--acq", type=float, default=10.0, help="segundos de adquisicion en cada posicion")
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--feed", type=float, default=2.0)
    ap.add_argument("--decimation", type=int, default=2, help="M120 P: 2 = 1 kHz")
    ap.add_argument("--settle", type=float, default=0.5, help="tiempo dentro de 5 um antes de empezar a adquirir")
    ap.add_argument("--abort-um", type=float, default=200.0)
    ap.add_argument("--from-dir")
    args = ap.parse_args()

    if args.from_dir:
        analyze(args.from_dir)
        return

    out_dir = os.path.join(ROOT, "mediciones", datetime.date.today().isoformat(),
                           "barrido_ki_" + datetime.datetime.now().strftime("%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    original = open(HW_CONFIG).read()
    meta = {"fecha": datetime.datetime.now().isoformat(timespec="seconds"), "param": args.param,
            "posiciones": args.positions, "acq_s": args.acq, "decimation": args.decimation, "valores": []}
    print(f"{args.param}: {', '.join(f'{v:g}' for v in args.values)}  ->  resultados en {out_dir}")
    with open(os.path.join(out_dir, "carga.log"), "w") as log:
        try:
            for v in args.values:
                try:
                    rec = run_value(args, v, out_dir, log)
                except (RuntimeError, TimeoutError) as e:
                    print(f"   ERROR con {args.param} = {v:g}: {e}")
                    rec = {"param": args.param, "value": v, "archivos": {}, "ok": False, "error": str(e)}
                meta["valores"].append(rec)
                json.dump(meta, open(os.path.join(out_dir, "barrido.json"), "w"), indent=1, default=float)
        except KeyboardInterrupt:
            print("\ninterrumpido")
        finally:
            open(HW_CONFIG, "w").write(original)
            print("\nsrc/hw_config.h restaurado; cargando el firmware original...")
            try:
                build_and_upload(args.port, log)
                print("firmware original cargado (hace falta M56 antes de mover)")
            except RuntimeError as e:
                print(f"NO se pudo recargar el firmware original: {e}")
    analyze(out_dir)


if __name__ == "__main__":
    main()
