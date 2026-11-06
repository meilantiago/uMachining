#!/usr/bin/env python3
"""Grafico prolijo de una escalera de tools/escalera_nm.py (para publicar).

    python3 tools/plot_escalera.py mediciones/2026-11-06/escalera_094219.csv
    python3 tools/plot_escalera.py <csv> --tema oscuro --suavizado-ms 100

Genera <csv sin extension>_claro.png y _oscuro.png (o solo el tema pedido). Textos en ingles.
El nivel de cada escalon descarta su primer segundo, igual que escalera_nm.py.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

TEMAS = {
    "claro": dict(fondo="#fcfcfb", texto="#0b0b0b", texto2="#52514e", tenue="#8a8984",
                  grilla="#e8e7e3", serie="#2a78d6"),
    "oscuro": dict(fondo="#1a1a19", texto="#f0efec", texto2="#c3c2b7", tenue="#8f8e86",
                   grilla="#2e2e2c", serie="#3987e5"),
}


def leer(ruta):
    with open(ruta) as f:
        cabecera = f.readline().lstrip("# ").strip()
    d = np.genfromtxt(ruta, delimiter=",", skip_header=2)
    # tramo -1: paquetes de antes del reposo (p.ej. el viaje hasta --pos si la telemetria ya estaba prendida)
    d = d[d[:, 1] >= 0]
    return cabecera, d[:, 0] - d[0, 0], d[:, 1].astype(int), d[:, 2], d[:, 3]


def niveles(t, tramo, pos):
    out = []
    for k in range(tramo.max() + 1):
        sel = tramo == k
        if not sel.any():
            continue
        t0, t1 = t[sel][0], t[sel][-1]
        m = sel & (t > t0 + 1.0)
        out.append(dict(k=k, t0=t0, t1=t1, nivel=pos[m].mean(), desvio=pos[m].std()))
    return out


def suavizar(t, x, ventana_s):
    """Media movil centrada en tiempo (tolera paquetes perdidos)."""
    c = np.concatenate([[0.0], np.cumsum(x)])
    i0 = np.searchsorted(t, t - ventana_s / 2)
    i1 = np.searchsorted(t, t + ventana_s / 2, side="right")
    return (c[i1] - c[i0]) / np.maximum(i1 - i0, 1)


def graficar(ruta, tema, suav_ms, salida):
    c = TEMAS[tema]
    _, t, tramo, objetivo, pos = leer(ruta)
    niv = niveles(t, tramo, pos)
    reposo = niv[0]
    pos_suave = suavizar(t, pos, suav_ms * 1e-3)

    plt.rcParams.update({
        "font.family": "Ubuntu", "font.size": 12,
        "axes.edgecolor": c["grilla"], "axes.labelcolor": c["texto2"],
        "xtick.color": c["texto2"], "ytick.color": c["texto2"],
        "text.color": c["texto"],
    })
    fig = plt.figure(figsize=(12, 6.6), dpi=200, facecolor=c["fondo"])
    ax = fig.add_axes([0.075, 0.15, 0.80, 0.64], facecolor=c["fondo"])

    # datos: crudo fino y transparente, media movil de 2 px, objetivo punteado neutro
    ax.plot(t, pos, color=c["serie"], lw=0.6, alpha=0.30, solid_joinstyle="round",
            label="Position, raw samples (1 kHz)", zorder=2)
    ax.plot(t, pos_suave, color=c["serie"], lw=2.0, solid_capstyle="round",
            label=f"Position, {suav_ms:g} ms average", zorder=3)
    ax.step(t, objetivo, where="post", color=c["texto2"], lw=1.3, ls=(0, (4, 3)),
            label="Commanded target", zorder=4)

    # etiquetas selectivas: el salto medido sobre cada escalon
    prev = reposo["nivel"]
    for n in niv[1:]:
        salto = n["nivel"] - prev
        prev = n["nivel"]
        xm = (n["t0"] + n["t1"]) / 2
        tope = np.percentile(pos[(t >= n["t0"]) & (t <= n["t1"])], 99.5)   # encima del ruido crudo
        ax.annotate(f"+{salto:.1f} nm", (xm, tope), xytext=(0, 6), textcoords="offset points",
                    ha="center", va="bottom", fontsize=12.5, fontweight="bold", color=c["texto"],
                    zorder=5)
    ax.annotate(f"at rest  σ = {reposo['desvio']:.1f} nm", ((reposo["t0"] + reposo["t1"]) / 2, reposo["nivel"]),
                xytext=(0, 15), textcoords="offset points", ha="center", va="bottom", fontsize=12,
                color=c["texto2"], zorder=5)

    # ejes recesivos
    ymin = np.floor((min(pos.min(), objetivo.min()) - 4) / 10) * 10
    ymax = np.ceil((max(pos.max(), objetivo.max()) + 10) / 10) * 10
    ax.set_ylim(ymin, ymax)
    ax.set_xlim(t[0], t[-1])
    ax.set_yticks(np.arange(ymin, ymax + 1, 10))
    ax.grid(axis="y", color=c["grilla"], lw=0.8, zorder=0)
    ax.tick_params(length=0, pad=6)
    for s in ("top", "right", "left"):
        ax.spines[s].set_visible(False)
    ax.spines["bottom"].set_color(c["tenue"])
    ax.set_xlabel("Time [s]", labelpad=8)
    ax.set_ylabel("Carriage position [nm]", labelpad=8)

    # etiquetas directas a la derecha del ultimo escalon (ademas de la leyenda)
    ult = niv[-1]
    ax.text(1.01, (ult["nivel"] - ymin) / (ymax - ymin), "measured", transform=ax.transAxes,
            color=c["serie"], fontsize=11.5, va="bottom", fontweight="bold")
    ax.text(1.01, (objetivo[-1] - ymin) / (ymax - ymin), "target", transform=ax.transAxes,
            color=c["texto2"], fontsize=11.5, va="top")

    leg = ax.legend(loc="upper left", frameon=False, fontsize=11, handlelength=2.4,
                    labelcolor=c["texto2"])
    for h in leg.legend_handles:
        h.set_alpha(1.0)

    # titulo, subtitulo y nota
    fig.text(0.075, 0.925, "Five 10 nm steps in closed loop", fontsize=21, fontweight="bold",
             color=c["texto"])
    fig.text(0.075, 0.87, "1.8° stepper on a 5 mm lead screw  ·  MT6835 magnetic encoder on a 60-magnet ring"
             "  ·  carriage at 30 mm", fontsize=12.5, color=c["texto2"])
    fig.text(0.075, 0.045, "Position measured by the axis's own encoder: this shows resolution, not accuracy. "
             "Step size = mean of each plateau (first second excluded).", fontsize=10.5, color=c["tenue"])

    fig.savefig(salida, facecolor=c["fondo"])
    plt.close(fig)
    print(salida)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--tema", choices=["claro", "oscuro", "ambos"], default="ambos")
    ap.add_argument("--suavizado-ms", type=float, default=50.0)
    args = ap.parse_args()
    base = os.path.splitext(args.csv)[0]
    temas = ["claro", "oscuro"] if args.tema == "ambos" else [args.tema]
    for tema in temas:
        graficar(args.csv, tema, args.suavizado_ms, f"{base}_{tema}.png")


if __name__ == "__main__":
    main()
