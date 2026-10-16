#!/usr/bin/env python3
"""Figuras de analisis de errores para el reporte de una prueba de lazo cerrado.

Lee el .npz de tools/closed_loop_test.py y genera, en <carpeta del npz>/reporte/:

  fig1_anatomia.png      un cuarto de segundo a velocidad de crucero: que es cada variable
  fig2_lazo_integrador.png  de donde sale el error constante (limite del integrador)
  fig3_presupuesto.png   descomposicion del error de seguimiento en componentes
  fig4_iman.png          mecanismo de la ondulacion por iman (velocidad con cuentas crudas)
  fig5_transitorios.png  arranque, frenado y reposo

y imprime los numeros que usa el reporte.

Uso:
    python3 tools/analisis_errores.py mediciones/2026-10-16/lazo_cerrado_20261016_093928.npz
    python3 tools/analisis_errores.py --comparar a_V0F0.npz b_V1F0.npz ...   # fig6_comparacion.png
    python3 tools/analisis_errores.py --metodo antes.npz despues.npz          # fig7_metodo_plegado.png
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import closed_loop_test as T  # estilo y constantes del montaje
import matplotlib.pyplot as plt

PITCH, PER = T.PITCH_MM, T.ENC_PERIOD_MM
POS_KP, POS_KI_LIMIT = 60.0, np.pi * 0.5          # src/hw_config.h, servo_controller.cpp
RAD_PER_MM = 2 * np.pi / PITCH
BLUE, ORANGE = T.DIR_COLOR[+1], T.DIR_COLOR[-1]
AQUA = "#1baf7a"


def load(path):
    z = np.load(path, allow_pickle=False)
    d = {k: z[k] for k in z.files if k != "meta"}
    meta = json.loads(str(z["meta"]))
    T.derive(d, meta)
    t = d["t_s"]
    # velocidad a partir de la posicion corregida por la tabla (derivada centrada de 8 ms)
    h = 4
    vp = np.full_like(t, np.nan)
    vp[h:-h] = (d["pos_mm"][2 * h:] - d["pos_mm"][:-2 * h]) / (t[2 * h:] - t[:-2 * h])
    d["vel_tabla_mm_s"] = vp
    return d, meta


def save(fig, out_dir, name):
    p = os.path.join(out_dir, name)
    fig.savefig(p, dpi=110)
    plt.close(fig)
    print("  " + p)


def fig_anatomia(d, out_dir):
    leg0 = d["leg"] == 0
    i0 = np.where(leg0 & d["cruise"] & (d["pos_mm"] > 120.0))[0][0]
    s = slice(i0, i0 + 250)
    t = (d["t_s"][s] - d["t_s"][i0]) * 1000
    rows = [
        ("objetivo y posición\n[mm]", None),
        ("error de seguimiento\n[µm]", "err_um"),
        ("velocidad [mm/s]", None),
        ("salida del lazo\n[° eléctricos]", "out_deg"),
        ("ángulo crudo del\nencoder [°]", "enc_deg"),
    ]
    fig, axs = plt.subplots(len(rows), 1, figsize=(13, 12), sharex=True, gridspec_kw=dict(hspace=0.15))
    a = axs[0]
    a.plot(t, d["target_mm"][s], color=T.MUTED, lw=3, label="objetivo (perfil trapezoidal)")
    a.plot(t, d["pos_mm"][s], color=BLUE, lw=1.2, label="posición (encoder + tabla)")
    a.legend(loc="upper left", fontsize=8)
    a.text(0.99, 0.08, "a esta escala no se distinguen: la diferencia es de ~13 µm",
           transform=a.transAxes, ha="right", color=T.INK2, fontsize=8)
    axs[1].plot(t, d["err_um"][s], color=BLUE, lw=1)
    axs[1].axhline(np.mean(d["err_um"][s]), color=T.INK2, lw=0.8, ls=(0, (3, 2)))
    axs[1].text(0.99, 0.9, f"media {np.mean(d['err_um'][s]):.1f} µm", transform=axs[1].transAxes,
                ha="right", color=T.INK2, fontsize=8)
    axs[2].plot(t, d["vel_mm_s"][s], color=BLUE, lw=1.2, label="la que usa el lazo (cuentas crudas, filtro 4 ms)")
    axs[2].plot(t, d["vel_tabla_mm_s"][s], color=AQUA, lw=1.2, label="derivada de la posición corregida (8 ms)")
    axs[2].axhline(2.0, color=T.MUTED, lw=0.8)
    axs[2].legend(loc="upper left", fontsize=8, ncol=2)
    axs[3].plot(t, d["out_deg"][s], color=BLUE, lw=1)
    T.limit_lines(axs[3])
    axs[3].set_ylim(-5, 90)
    axs[4].plot(t, d["enc_deg"][s], ".", color=BLUE, ms=2)
    for k, (lab, _) in enumerate(rows):
        axs[k].set_ylabel(lab)
    axs[-1].set_xlabel("tiempo [ms]   (ida 1, desde x ≈ 120 mm, 1 muestra por ms)")
    axs[-1].set_xlim(t[0], t[-1])
    # marcar periodos del encoder
    wraps = np.where(np.diff(d["enc_deg"][s]) < -180)[0]
    for w in wraps:
        for a in axs:
            a.axvline(t[w], color=T.AXIS, lw=0.7)
    axs[4].text(t[wraps[0]] if len(wraps) else 0, 1.02, " cada línea gris = un período del encoder (2 imanes, 167 µm)",
                transform=axs[4].get_xaxis_transform(), fontsize=8, color=T.INK2)
    fig.suptitle("Anatomía de un cuarto de segundo a 2 mm/s: qué es cada variable", x=0.01, ha="left",
                 fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.1, right=0.93, top=0.95, bottom=0.05)
    save(fig, out_dir, "fig1_anatomia.png")


def fig_integrador(d, out_dir):
    m = (d["leg"] == 0) & d["cruise"]
    v_rad = np.mean(np.abs(d["vel"][m]))
    p_term = v_rad - POS_KI_LIMIT
    err_pred = p_term / POS_KP / RAD_PER_MM * 1000
    err_meas = np.mean(d["err_um"][m])
    fig, ax = plt.subplots(figsize=(11, 3.4))
    ax.barh([0], [POS_KI_LIMIT], color="#86b6ef", height=0.5)
    ax.barh([0], [p_term], left=[POS_KI_LIMIT], color=BLUE, height=0.5)
    ax.text(POS_KI_LIMIT / 2, 0, f"integral\n{POS_KI_LIMIT:.3f} rad/s\n(clavada en su límite π/2)",
            ha="center", va="center", fontsize=9)
    ax.text(POS_KI_LIMIT + p_term / 2, 0, f"proporcional\n60 × error\n{p_term:.3f} rad/s",
            ha="center", va="center", fontsize=9, color="white")
    ax.annotate(f"velocidad necesaria para 2 mm/s: {v_rad:.3f} rad/s", xy=(v_rad, 0.27), xytext=(v_rad + 0.05, 0.5),
                fontsize=9, arrowprops=dict(arrowstyle="-", color=T.INK2, lw=0.8))
    ax.set_yticks([])
    ax.set_xlim(0, 4.2)
    ax.set_ylim(-0.5, 0.75)
    ax.set_xlabel("velocidad que pide el lazo de posición [rad/s de motor]")
    ax.grid(axis="y", visible=False)
    ax.spines["left"].set_visible(False)
    fig.suptitle(f"El error constante: la integral no alcanza y la proporcional necesita "
                 f"{p_term:.3f} / 60 = {err_pred:.1f} µm de error (medido: {err_meas:.1f} µm)",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.03, right=0.97, top=0.82, bottom=0.2)
    save(fig, out_dir, "fig2_lazo_integrador.png")


def fold_component(phase, y, n_bins):
    """Forma media de y en funcion de una fase 0..1 (lo que se repite con ese periodo)."""
    b = np.minimum((phase * n_bins).astype(int), n_bins - 1)
    cnt = np.bincount(b, minlength=n_bins)
    mean = np.bincount(b, y, minlength=n_bins) / np.maximum(cnt, 1)
    return mean, mean[b]


def budget(d, k):
    """Separa el error de seguimiento del tramo k (a velocidad de crucero) en componentes: la
    constante, las que se repiten con un periodo fisico conocido (por plegado) y el resto."""
    m = (d["leg"] == k) & d["cruise"]
    y = d["err_um"][m].copy()
    x = d["pos_mm"][m]
    comps = [("constante", y.mean())]
    r = y - y.mean()
    for lab, phase, nb in (("por imán (83 µm)", (d["enc_deg"][m] % 180) / 180, 36),
                           ("por paso del motor (25 µm)", (x % 0.025) / 0.025, 10),
                           ("por ciclo eléctrico (100 µm)", (x % 0.1) / 0.1, 20),
                           ("por vuelta de motor (5 mm)", (x % PITCH) / PITCH, 50)):
        shape, rec = fold_component(phase, r, nb)
        comps.append((lab, np.ptp(shape) / 2))
        r = r - rec
    comps.append(("resto (aleatorio + picos), desvío", r.std()))
    return comps


def fig_presupuesto(d, out_dir):
    res = {name: budget(d, k) for k, name in ((0, "ida 1"), (1, "vuelta 1"))}
    labels = [c[0] for c in res["ida 1"]]
    fig, ax = plt.subplots(figsize=(12, 4.8))
    yy = np.arange(len(labels))
    for j, (name, color) in enumerate((("ida 1", BLUE), ("vuelta 1", ORANGE))):
        vals = [abs(v) for _, v in res[name]]
        ax.barh(yy + (j - 0.5) * 0.36, vals, height=0.34, color=color, label=name)
        for yi, v in zip(yy, vals):
            ax.text(v + 0.15, yi + (j - 0.5) * 0.36, f"{v:.1f}", va="center", fontsize=8, color=T.INK2)
    ax.set_yticks(yy)
    ax.set_yticklabels([("constante (atraso)" if l == "constante" else l) for l in labels])
    ax.invert_yaxis()
    ax.set_xlabel("µm  (constante: valor medio; periódicas: amplitud = mitad del pico a pico; resto: desvío estándar)")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="lower right")
    fig.suptitle("¿De qué está hecho el error de seguimiento a 2 mm/s?", x=0.01, ha="left",
                 fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.25, right=0.97, top=0.88, bottom=0.14)
    save(fig, out_dir, "fig3_presupuesto.png")
    return {n: {l: round(float(v), 2) for l, v in c} for n, c in res.items()}


def fig_iman(d, out_dir):
    edges = np.linspace(0, 360, 73)
    xc = 0.5 * (edges[1:] + edges[:-1])
    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True, gridspec_kw=dict(hspace=0.12))
    out = {}
    for k, ls in ((0, "-"), (1, (0, (4, 2)))):
        m = (d["leg"] == k) & d["cruise"] & np.isfinite(d["vel_tabla_mm_s"])
        name = "ida 1" if k == 0 else "vuelta 1"
        coh_um = d["coh_deg"][m] / 360 * PER * 1000
        c, _, _ = T.binned(d["enc_deg"][m], coh_um, edges)
        vr, _, _ = T.binned(d["enc_deg"][m], np.abs(d["vel_mm_s"][m]), edges)
        vt, _, _ = T.binned(d["enc_deg"][m], np.abs(d["vel_tabla_mm_s"][m]), edges)
        e, _, _ = T.binned(d["enc_deg"][m], np.abs(d["err_um"][m]), edges)
        if k == 0:
            axs[0].plot(xc, c - np.mean(c), color=T.INK2, lw=1.8)
        axs[1].plot(xc, vr, color=BLUE, lw=1.8, ls=ls, label=f"{name}: la que ve el lazo (cuentas crudas)")
        axs[1].plot(xc, vt, color=AQUA, lw=1.8, ls=ls, label=f"{name}: movimiento real según la tabla")
        axs[2].plot(xc, e, color=BLUE if k == 0 else ORANGE, lw=1.8, ls=ls, label=name)
        out[name] = dict(no_lineal_pp_um=float(np.ptp(c)), vel_cruda_pp=float(np.ptp(vr)),
                         vel_tabla_pp=float(np.ptp(vt)), err_pp_um=float(np.ptp(e)))
    axs[0].set_ylabel("no linealidad del\nencoder crudo [µm]")
    axs[0].text(0.01, 0.9, "lo que la tabla de calibración le corrige al ángulo crudo (sale de la coherencia)",
                transform=axs[0].transAxes, fontsize=8, color=T.INK2)
    axs[1].axhline(2.0, color=T.MUTED, lw=0.8)
    axs[1].set_ylabel("|velocidad| [mm/s]")
    axs[1].legend(loc="upper right", fontsize=8, ncol=2)
    axs[2].set_ylabel("|error de\nseguimiento| [µm]")
    axs[2].legend(loc="upper right", fontsize=8)
    axs[-1].set_xlabel("ángulo crudo del encoder dentro del período [°]   (0–360° = 2 imanes = 167 µm)")
    axs[-1].set_xlim(0, 360)
    for a in axs:
        a.axvline(180, color=T.AXIS, lw=0.8)
    fig.suptitle("Ondulación por imán: el lazo de velocidad corrige una velocidad mal medida", x=0.01,
                 ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.1, right=0.98, top=0.94, bottom=0.07)
    save(fig, out_dir, "fig4_iman.png")
    return out


def fig_transitorios(d, out_dir):
    t = d["t_s"]
    leg = d["leg"]
    fig, axs = plt.subplots(2, 3, figsize=(15, 7), sharey="row", gridspec_kw=dict(wspace=0.06, hspace=0.12))
    m0 = np.where(leg == 0)[0]
    m1 = np.where(leg == 1)[0]
    windows = [("arranque (ida 1, X = 0)", m0[0] - 100, m0[0] + 900),
               ("frenado (ida 1, X = 255)", m0[-1] - 1300, m0[-1] - 300),
               ("reposo en X = 0 (al final: arranca la ida 2)", m1[-1] - 300, m1[-1] + 1200)]
    stats = {}
    for c, (title, a, b) in enumerate(windows):
        a, b = max(a, 0), min(b, len(t) - 1)
        tt = (t[a:b] - t[a]) * 1000
        axs[0, c].plot(tt, d["err_um"][a:b], color=BLUE, lw=1)
        axs[1, c].plot(tt, d["out_deg"][a:b], color=BLUE, lw=1)
        ax2 = axs[0, c]
        ax2.set_title(title)
        T.limit_lines(axs[1, c], label=(c == 2))
        axs[1, c].set_xlabel("tiempo [ms]")
        vt = np.abs(d["vt_mm_s"][a:b])
        acc = (vt > 0.005) & (vt < 1.95)
        for ax in axs[:, c]:
            ax.fill_between(tt, 0, 1, where=acc, transform=ax.get_xaxis_transform(), color=T.MUTED,
                            alpha=0.12, lw=0)
        stats[title] = float(np.max(np.abs(d["err_um"][a:b])))
    axs[0, 0].set_ylabel("error de seguimiento [µm]")
    axs[1, 0].set_ylabel("salida [° el.]")
    axs[1, 0].set_ylim(-90, 90)
    fig.suptitle("Transitorios: sombreado = el perfil está acelerando o frenando (5 mm/s²)", x=0.01, ha="left",
                 fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.07, right=0.95, top=0.88, bottom=0.08)
    save(fig, out_dir, "fig5_transitorios.png")
    return stats


def numbers(d, meta):
    print("\n--- numeros para el reporte ---")
    for k in range(len(meta["legs"])):
        m = (d["leg"] == k) & d["cruise"]
        e = d["err_um"][m]
        print(f"{meta['legs'][k]['name']:9s} crucero: err media {e.mean():+.2f} µm, desvio {e.std():.2f} µm, "
              f"rango {e.min():+.1f}..{e.max():+.1f} µm, vel media {np.mean(np.abs(d['vel_mm_s'][m])):.4f} mm/s")
    v = 2.0 * RAD_PER_MM
    print(f"error constante predicho por el limite del integrador: {(v - POS_KI_LIMIT) / POS_KP / RAD_PER_MM * 1000:.2f} µm")
    for name, cond in (("reposo X=255", (d["leg"] == -1) & (d["target_mm"] > 254)),
                       ("reposo X=0", (d["leg"] == -1) & (d["target_mm"] < 1))):
        cond &= np.abs(d["vt_mm_s"]) < 1e-6
        e = d["err_um"][cond]
        print(f"{name}: desvio {e.std():.2f} µm, pico a pico {np.ptp(e):.2f} µm, salida media {d['out_deg'][cond].mean():+.1f}°")


CONFIG_NAMES = {"V0F0": "antes (cuentas crudas, sin anticipada)", "V1F0": "solo velocidad con la tabla (B1)",
                "V0F1": "solo realimentación anticipada (A1)", "V1F1": "las dos (B1 + A1)"}
CONFIG_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]


def compare(paths, out_dir):
    """Compara varias corridas (una por configuracion de M58): presupuesto del error, picos y
    forma por iman. Usa la ida 1 y la vuelta 1 de cada archivo."""
    runs = []
    for p in paths:
        d, meta = load(p)
        tag = os.path.splitext(os.path.basename(p))[0].split("_")[-1]
        runs.append((tag, d, meta))
    os.makedirs(out_dir, exist_ok=True)
    rows = []
    for tag, d, meta in runs:
        for k in range(len(meta["legs"])):
            m = d["leg"] == k
            mc = m & d["cruise"]
            b = dict(budget(d, k))
            rows.append(dict(cfg=tag, tramo=meta["legs"][k]["name"], ok=meta["legs"][k]["ok"],
                             max_err=float(np.max(np.abs(d["err_um"][m]))),
                             rms=float(np.sqrt(np.mean(d["err_um"][mc] ** 2))),
                             const=float(b["constante"]), iman=float(b["por imán (83 µm)"]),
                             vuelta=float(b["por vuelta de motor (5 mm)"]),
                             resto=float(b["resto (aleatorio + picos), desvío"]),
                             out_med=float(np.mean(d["out_deg"][mc])),
                             max_out=float(np.max(np.abs(d["out_deg"][m]))),
                             sat=int(np.sum(np.abs(d["out_deg"][m]) > T.OUT_LIMIT_DEG - 0.5)),
                             out_pp_iman=float(np.ptp(T.binned(d["enc_deg"][mc] % 180, d["out_deg"][mc],
                                                               np.linspace(0, 180, 37))[0]))))
    print(f"{'config':5s} {'tramo':9s} {'ok':3s} {'max|err|':>8s} {'rms':>6s} {'const':>6s} {'imán±':>6s} "
          f"{'vuelta±':>7s} {'resto':>6s} {'out med':>7s} {'max|out|':>8s} {'satur':>6s} {'out pp imán':>11s}")
    for r in rows:
        print(f"{r['cfg']:5s} {r['tramo']:9s} {'si' if r['ok'] else 'NO':3s} {r['max_err']:8.1f} {r['rms']:6.1f} "
              f"{r['const']:+6.1f} {r['iman']:6.1f} {r['vuelta']:7.1f} {r['resto']:6.1f} {r['out_med']:+7.1f} "
              f"{r['max_out']:8.1f} {r['sat']:6d} {r['out_pp_iman']:11.1f}")

    # figura: componentes del error (ida 1) y forma por iman
    comp_keys = [("const", "constante (atraso)"), ("iman", "por imán, amplitud"), ("vuelta", "por vuelta, amplitud"),
                 ("resto", "resto, desvío"), ("max_err", "máximo |error|")]
    fig = plt.figure(figsize=(15, 10))
    gs = fig.add_gridspec(2, len(runs), height_ratios=[1.15, 1], hspace=0.35, wspace=0.08)
    ax = fig.add_subplot(gs[0, :])
    yy = np.arange(len(comp_keys))
    h = 0.8 / len(runs)
    for j, (tag, d, meta) in enumerate(runs):
        r = [x for x in rows if x["cfg"] == tag and x["tramo"] == "ida 1"][0]
        vals = [abs(r[k]) for k, _ in comp_keys]
        ax.barh(yy + (j - (len(runs) - 1) / 2) * h, vals, height=h * 0.9, color=CONFIG_COLORS[j % 4],
                label=CONFIG_NAMES.get(tag, tag))
        for yi, v in zip(yy, vals):
            ax.text(v + 0.2, yi + (j - (len(runs) - 1) / 2) * h, f"{v:.1f}", va="center", fontsize=7.5, color=T.INK2)
    ax.set_yticks(yy)
    ax.set_yticklabels([l for _, l in comp_keys])
    ax.invert_yaxis()
    ax.set_xlabel("µm (ida 1, velocidad de crucero salvo el máximo)")
    ax.grid(axis="y", visible=False)
    ax.legend(loc="center right", fontsize=8)
    ax.set_title("Componentes del error de seguimiento por configuración")
    edges = np.linspace(0, 360, 73)
    xc = 0.5 * (edges[1:] + edges[:-1])
    for j, (tag, d, meta) in enumerate(runs):
        a = fig.add_subplot(gs[1, j], sharey=None if j == 0 else fig.axes[1])
        for k, color in ((0, BLUE), (1, ORANGE)):
            m = (d["leg"] == k) & d["cruise"]
            e, _, _ = T.binned(d["enc_deg"][m], d["err_um"][m], edges)
            a.plot(xc, e, color=color, lw=1.8, label=meta["legs"][k]["name"])
        a.axhline(0, color=T.AXIS, lw=0.8)
        a.set_title(CONFIG_NAMES.get(tag, tag), fontsize=9)
        a.set_xlabel("ángulo del encoder [°]")
        a.set_xlim(0, 360)
        a.set_xticks([0, 180, 360])
        if j == 0:
            a.set_ylabel("error plegado sobre\n2 imanes [µm]")
            a.legend(fontsize=8, loc="center left")
        else:
            a.tick_params(labelleft=False)
    fig.suptitle("Antes y después: velocidad con la tabla (B1) y realimentación anticipada (A1)", x=0.01, ha="left",
                 fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.13, right=0.98, top=0.9, bottom=0.07)
    save(fig, out_dir, "fig6_comparacion.png")
    return rows


def fold_scan(x, y, periods, n_bins=20):
    """Para cada periodo de prueba P, amplitud (mitad del pico a pico) de la forma media de y
    plegada sobre x modulo P. Solo da algo si y tiene una componente que se repite con P."""
    amp = np.empty(len(periods))
    for i, P in enumerate(periods):
        b = np.minimum(((x % P) / P * n_bins).astype(int), n_bins - 1)
        cnt = np.bincount(b, minlength=n_bins)
        mean = np.bincount(b, y, minlength=n_bins) / np.maximum(cnt, 1)
        amp[i] = np.ptp(mean) / 2
    return amp


def fig_metodo(path_antes, path_despues, out_dir):
    """Figura del metodo de plegado: datos crudos, superposicion de ciclos y barrido de periodos."""
    runs = [(load(path_antes)[0], "antes (V0 F0)", T.DIR_COLOR[+1]), (load(path_despues)[0], "B1 + A1", AQUA)]
    d0 = runs[0][0]
    m0 = (d0["leg"] == 0) & d0["cruise"]
    x0, e0 = d0["pos_mm"][m0], d0["err_um"][m0]
    fig = plt.figure(figsize=(15, 11))
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.2], hspace=0.42, wspace=0.15)

    # (a) un milimetro de datos crudos con los bordes de cada iman
    a = fig.add_subplot(gs[0, :])
    sel = (x0 > 120) & (x0 < 121)
    a.plot(x0[sel], e0[sel], color=BLUE, lw=1)
    enc = d0["enc_deg"][m0][sel]
    for xb in x0[sel][1:][np.diff(enc % 180) < -90]:
        a.axvline(xb, color=T.AXIS, lw=0.8)
    a.set_title("(a) Datos crudos: 1 mm de la ida a 2 mm/s, antes de los cambios. Líneas grises = borde de cada imán (cada 83 µm)")
    a.set_xlabel("posición [mm]")
    a.set_ylabel("error [µm]")

    # (b) superposicion de ciclos de iman
    b = fig.add_subplot(gs[1, 0])
    ph = (d0["enc_deg"][m0] % 180) / 180 * 83.3
    cyc = np.floor(d0["enc_deg"][m0] / 180).astype(int) + np.round(x0 / PER).astype(int) * 2
    ids = np.unique(cyc)
    rng = np.random.default_rng(0)
    for c in rng.choice(ids, 150, replace=False):
        mm = cyc == c
        o = np.argsort(ph[mm])
        b.plot(ph[mm][o], e0[mm][o], color=T.MUTED, lw=0.4, alpha=0.35)
    edges = np.linspace(0, 83.3, 37)
    mean, _, _ = T.binned(ph, e0, edges)
    b.plot(0.5 * (edges[1:] + edges[:-1]), mean, color=BLUE, lw=2.5, label=f"promedio de los {len(ids)} imanes recorridos")
    b.legend(loc="upper right", fontsize=8)
    b.set_title("(b) Plegado: 150 imanes superpuestos (gris) y el promedio de todos")
    b.set_xlabel("posición dentro del imán [µm]")
    b.set_ylabel("error [µm]")

    # (c) lo mismo plegando con un periodo equivocado (90 µm)
    c = fig.add_subplot(gs[1, 1], sharey=b)
    P = 0.090
    ph2 = (x0 % P) / P * 90
    mean2, _, _ = T.binned(ph2, e0, np.linspace(0, 90, 37))
    for k in rng.choice(np.unique(np.floor(x0 / P)), 150, replace=False):
        mm = np.floor(x0 / P) == k
        o = np.argsort(ph2[mm])
        c.plot(ph2[mm][o], e0[mm][o], color=T.MUTED, lw=0.4, alpha=0.35)
    c.plot(np.linspace(0, 90, 37)[:-1] + 1.25, mean2, color=CRITICAL_SAFE, lw=2.5, label="promedio")
    c.legend(loc="upper right", fontsize=8)
    c.set_title("(c) El mismo plegado con un período que no existe (90 µm): el promedio queda plano")
    c.set_xlabel("posición dentro de un tramo de 90 µm [µm]")
    c.tick_params(labelleft=False)

    # (d) barrido fino de periodos. El pico de cada periodo es muy angosto: plegando L = 255 mm,
    # un error dP en el periodo corre la fase L/P * dP, asi que el ancho es ~P^2/L = 0.03 um.
    a = fig.add_subplot(gs[2, :])
    periods = np.arange(0.070, 0.105, 0.000004)
    for d, lab, color in runs:
        m = (d["leg"] == 0) & d["cruise"]
        amp = fold_scan(d["pos_mm"][m], d["err_um"][m] - d["err_um"][m].mean(), periods)
        a.plot(periods * 1000, amp, color=color, lw=1.2, label=lab)
    for P, lab in ((75, "3 pasos (75 µm)"), (250 / 3, "1 imán (83,3 µm)"), (100, "ciclo eléctrico = 4 pasos (100 µm)")):
        a.axvline(P, color=T.AXIS, lw=0.8, ls=(0, (3, 2)))
        a.text(P, 1.0, " " + lab, transform=a.get_xaxis_transform(), fontsize=8, color=T.INK2, va="bottom")
    a.set_xlabel("período de prueba P [µm]   (paso del barrido: 0,004 µm)")
    a.set_ylabel("amplitud de lo que\nse repite cada P [µm]")
    a.set_title("(d) Barrido: se pliega con cada período entre 70 y 105 µm; solo aparecen picos donde hay una causa física",
                pad=16)
    a.legend(loc="upper left", fontsize=9)
    a.set_xlim(70, 105)
    a.set_ylim(0, None)
    fig.suptitle("Cómo se separan las contribuciones: plegado (promedio sincrónico) sobre cada período físico",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.06, right=0.98, top=0.93, bottom=0.06)
    save(fig, out_dir, "fig7_metodo_plegado.png")


CRITICAL_SAFE = "#6250d6"


def main():
    if sys.argv[1] == "--metodo":
        fig_metodo(sys.argv[2], sys.argv[3], os.path.join(os.path.dirname(sys.argv[2]), "reporte"))
        return
    if sys.argv[1] == "--comparar":
        paths = sys.argv[2:]
        compare(paths, os.path.join(os.path.dirname(paths[0]), "reporte"))
        return
    path = sys.argv[1]
    d, meta = load(path)
    out_dir = os.path.join(os.path.dirname(path), "reporte")
    os.makedirs(out_dir, exist_ok=True)
    print("figuras:")
    fig_anatomia(d, out_dir)
    fig_integrador(d, out_dir)
    peaks = fig_presupuesto(d, out_dir)
    iman = fig_iman(d, out_dir)
    tr = fig_transitorios(d, out_dir)
    numbers(d, meta)
    print("presupuesto:", peaks)
    print("ondulacion por iman:", {k: {a: round(b, 2) for a, b in v.items()} for k, v in iman.items()})
    print("transitorios max|err|:", {k: round(v, 1) for k, v in tr.items()})


if __name__ == "__main__":
    main()
