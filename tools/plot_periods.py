#!/usr/bin/env python3
"""Grafica la rotacion del campo medida por el MT6835 en ventanas de 2 periodos
(4 imanes) a lo largo de toda la vuelta, para la ida y para la vuelta, a partir de un
CSV de tools/encoder_sweep.py.

En cada ventana se superponen las dos vueltas del barrido en el mismo sentido: si una
deformacion aparece igual en ambas, es propia de esa zona de la rueda (imanes, montaje).

Uso:
    python3 tools/plot_periods.py mediciones/encoder_sweep_XXXX.csv
    python3 tools/plot_periods.py archivo.csv --periods-per-window 1   # ventanas de 2 imanes

Los imanes se numeran desde la posicion del motor al empezar el barrido (iman 0).
Para ubicarlos fisicamente, marcar en la rueda el iman que esta frente al sensor antes
de correr el barrido.
"""
import argparse
import os
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def nan_at_wraps(y):
    y = np.array(y, dtype=float)
    if len(y) > 1:
        y[1:][np.abs(np.diff(y)) > 180] = np.nan
    return y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--magnets", type=int, default=60)
    ap.add_argument("--pitch", type=float, default=3.0, help="paso entre imanes [mm]")
    ap.add_argument("--periods-per-window", type=int, default=2)
    args = ap.parse_args()

    d = np.genfromtxt(args.csv, delimiter=",", names=True)
    circumference = args.magnets * args.pitch
    period_mm = 2 * args.pitch
    win_mm = args.periods_per_window * period_mm
    n_win = int(round(circumference / win_mm))

    mech = d["motor_deg"] % 360.0
    x = mech / 360.0 * circumference                 # posicion dentro de la vuelta [mm]
    rev = np.floor(d["motor_deg"] / 360.0).astype(int)
    enc = wrap180(d["enc_deg"])
    ideal = wrap180(d["ideal_deg"])
    t = d["t_s"]

    # descartar las rampas de arranque/frenado de cada sentido (1 s)
    steady = np.zeros(len(d), bool)
    for s in (+1, -1):
        m = d["dir"] == s
        if m.any():
            steady |= m & (t > t[m][0] + 1.0) & (t < t[m][-1] - 1.0)

    base = os.path.splitext(args.csv)[0]
    fig, ax = plt.subplots(n_win, 2, figsize=(14, 2.6 * n_win), sharey=True)
    ax = np.atleast_2d(ax)
    colors = {0: "tab:blue", 1: "tab:green"}
    summary = []

    for w in range(n_win):
        x0, x1 = w * win_mm, (w + 1) * win_mm
        mag0 = int(round(x0 / args.pitch))
        mag1 = int(round(x1 / args.pitch)) - 1
        for col, (s, name) in enumerate(((+1, "ida"), (-1, "vuelta"))):
            a = ax[w, col]
            sel_all = steady & (d["dir"] == s) & (x >= x0) & (x < x1)
            for r in sorted(set(rev[sel_all])):
                m = sel_all & (rev == r)
                o = np.argsort(x[m])
                xs, es, ids = x[m][o], enc[m][o], ideal[m][o]
                # la primera vuelta con puntos grandes y claros debajo, para que se vea aunque
                # la segunda caiga exactamente encima
                big = r == min(set(rev[sel_all]))
                a.plot(xs, es, "o" if big else ".", ms=4 if big else 1.5, alpha=0.35 if big else 1.0,
                       color=colors.get(r, "gray"), label=f"vuelta {r+1}")
                if r == min(set(rev[sel_all])):
                    a.plot(xs, nan_at_wraps(ids), "-", lw=0.8, color="tab:orange", label="ideal")
            wk = sel_all & (d["weak"] > 0)
            ov = sel_all & (d["overspeed"] > 0)
            a.plot(x[wk], enc[wk], "rx", ms=4, label="campo debil")
            a.plot(x[ov], enc[ov], "m+", ms=4, label="overspeed")

            err = d["error_deg"][sel_all]
            err_ok = err[np.abs(err) < 120]
            weak_pct = 100 * d["weak"][sel_all].mean() if sel_all.any() else 0
            rms = np.sqrt(np.mean(err_ok ** 2)) if len(err_ok) else np.nan
            flips = int(np.sum(np.abs(err) >= 120))
            summary.append((w, name, mag0, mag1, weak_pct, flips, rms))

            a.set_title(f"{name}: imanes {mag0}-{mag1} ({mech_deg(x0, circumference):.0f}-"
                        f"{mech_deg(x1, circumference):.0f} grados)  debil {weak_pct:.0f}%  "
                        f"error rms {rms:.0f}  muestras corridas {flips}", fontsize=8)
            a.set_xlim(x0, x1)
            a.set_ylim(-190, 190)
            for k in range(args.periods_per_window * 2 + 1):   # lineas en los bordes de cada iman
                a.axvline(x0 + k * args.pitch, color="gray", lw=0.4, alpha=0.5)
            a.grid(alpha=0.3)
            if w == 0:
                a.legend(fontsize=7, markerscale=3, loc="upper left")
        ax[w, 0].set_ylabel("angulo [grados]")
    ax[-1, 0].set_xlabel("posicion en la vuelta [mm]")
    ax[-1, 1].set_xlabel("posicion en la vuelta [mm]")
    fig.suptitle(os.path.basename(args.csv) + " - rotacion del campo por ventanas", y=1.0)
    fig.tight_layout()
    out = base + f"_periodos_{args.periods_per_window}.png"
    fig.savefig(out, dpi=80)

    print("ventana  sentido  imanes    campo_debil%  muestras_corridas_180  error_rms[grados encoder]")
    for w, name, m0, m1, wp, fl, rms in summary:
        print(f"  {w:2d}    {name:6s}  {m0:2d}-{m1:2d}      {wp:5.1f}          {fl:4d}               {rms:5.1f}")
    print(f"grafico en {os.path.abspath(out)}")


def mech_deg(x_mm, circumference):
    return x_mm / circumference * 360.0


if __name__ == "__main__":
    main()
