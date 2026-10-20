#!/usr/bin/env python3
"""Analisis para elegir POS_KI a partir de un barrido de tools/pid_sweep.py que solo cambia POS_KI.

Para cada valor de POS_KI separa cada reposo (uno despues de cada tramo) y cada tramo en
movimiento, y calcula:

  reposo     desvio y pico a pico del error en el reposo, sin el primer segundo despues de
             parar; si hay "caza" (ciclo limite contra la friccion estatica) y su frecuencia
  llegada    error a 0,1 / 0,5 / 1 s despues de que el perfil se detiene, y cuanto tarda en
             quedar dentro de 0,05 um
  movimiento RMS y maximo del error en cada tramo, atraso medio en crucero

Genera, en la carpeta del barrido:
  eleccion_pos_ki.png   reposo, llegada y movimiento contra POS_KI (un punto por repeticion)
  reposo_trazas.png     el error en cada reposo (filas = POS_KI, columnas = repeticiones)
  eleccion_pos_ki.csv   todas las metricas por reposo y por tramo

Uso:
    python3 tools/ki_eleccion.py mediciones/AAAA-MM-DD/barrido_pid_HHMMSS
"""
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import closed_loop_test as T  # noqa: E402  (estilo)
import pid_sweep as S  # noqa: E402  (carga del barrido)
import matplotlib.pyplot as plt  # noqa: E402

SKIP_S = 1.0          # se descarta el primer segundo despues de parar (llegada) para medir el reposo
SETTLE_UM = 0.05


def stops(d, meta):
    """Para cada tramo completo: (tramo, indice donde el perfil se detiene, indices del reposo que sigue)."""
    vt = np.abs(d["vt_mm_s"])
    leg = d["leg"]
    n = len(leg)
    out = []
    for L in meta["legs"]:
        if not L["ok"]:
            continue
        idx = np.where(leg == L["leg"])[0]
        moving = idx[vt[idx] > 1e-6]
        if len(moving) == 0:
            continue
        stop = moving[-1] + 1
        j = idx[-1] + 1
        while j < n and leg[j] == -1:
            j += 1
        out.append((L, stop, np.arange(stop, j)))
    return out


def hunting(t, e):
    """Frecuencia [Hz] y amplitud (mitad del pico a pico) si el error oscila; si no, (0, amplitud)."""
    x = e - e.mean()
    amp = np.ptp(x) / 2
    if amp < 0.02:
        return 0.0, amp
    zc = np.where((x[:-1] < -0.3 * amp) & (np.roll(x, -1)[:-1] >= -0.3 * amp))[0]
    up = np.where(np.diff(np.sign(x - 0)) > 0)[0]
    if len(up) < 3 or len(zc) < 2:
        return 0.0, amp
    f = (len(up) - 1) / (t[up[-1]] - t[up[0]])
    return f, amp


def analyze_config(meta, d):
    t, e = d["t_s"], d["err_um"]
    rests, moves = [], []
    for L, stop, w in stops(d, meta):
        tt = t[w] - t[stop]
        ew = e[w]
        rest = tt >= SKIP_S
        at = {}
        for ts in (0.1, 0.5, 1.0):
            k = np.searchsorted(tt, ts)
            at[ts] = float(ew[min(k, len(ew) - 1)])
        bad = np.where(np.abs(ew) > SETTLE_UM)[0]
        t_settle = float(tt[bad[-1]] + 0.001) if len(bad) and bad[-1] < len(ew) - 1 else (0.0 if not len(bad) else np.nan)
        f, amp = hunting(tt[rest], ew[rest]) if rest.sum() > 100 else (np.nan, np.nan)
        rests.append({"tramo": L["name"], "x_mm": L["dest"], "dur_s": float(tt[-1]),
                      "desvio": float(np.std(ew[rest])) if rest.sum() > 100 else np.nan,
                      "pp": float(np.ptp(ew[rest])) if rest.sum() > 100 else np.nan,
                      "media": float(np.mean(ew[rest])) if rest.sum() > 100 else np.nan,
                      "caza_hz": f, "caza_amp": amp,
                      "e_0.1": at[0.1], "e_0.5": at[0.5], "e_1.0": at[1.0], "t_asentado": t_settle,
                      "w": w, "stop": stop})
        m = d["leg"] == L["leg"]
        mc = m & d["cruise"]
        moves.append({"tramo": L["name"], "rms": float(np.sqrt(np.mean(e[mc] ** 2))) if mc.any() else np.nan,
                      "max": float(np.max(np.abs(e[m]))), "atraso": float(np.mean(e[mc])) if mc.any() else np.nan})
    return rests, moves


def main():
    out_dir = sys.argv[1]
    runs = S.load_dir(out_dir)
    rows = []
    for meta, d, _ in runs:
        if meta["estado"] != "ok":
            continue
        rests, moves = analyze_config(meta, d)
        rows.append((meta, d, rests, moves))
    # orden: POS_KI creciente; la referencia (base) al final
    rows.sort(key=lambda r: (r[0]["config"] in ("base", "base_final"), r[0]["pid"]["pos_ki"], r[0]["config"] == "base_final"))

    def name(meta):
        k = meta["pid"]["pos_ki"]
        return {"base": f"{k:g}\n(ref. inicio)", "base_final": f"{k:g}\n(ref. final)"}.get(meta["config"], f"{k:g}")

    # --- tabla ---
    with open(os.path.join(out_dir, "eleccion_pos_ki.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["config", "pos_ki", "tipo", "tramo", "x_mm", "desvio_um", "pico_a_pico_um", "media_um", "caza_hz",
                    "caza_amp_um", "e_0.1s_um", "e_0.5s_um", "e_1s_um", "t_asentado_0.05um_s", "rms_um", "max_um", "atraso_um"])
        for meta, d, rests, moves in rows:
            for r in rests:
                w.writerow([meta["config"], meta["pid"]["pos_ki"], "reposo", r["tramo"], r["x_mm"], f"{r['desvio']:.4f}",
                            f"{r['pp']:.4f}", f"{r['media']:+.4f}", f"{r['caza_hz']:.2f}", f"{r['caza_amp']:.4f}",
                            f"{r['e_0.1']:+.3f}", f"{r['e_0.5']:+.3f}", f"{r['e_1.0']:+.3f}", f"{r['t_asentado']:.3f}",
                            "", "", ""])
            for m in moves:
                w.writerow([meta["config"], meta["pid"]["pos_ki"], "movimiento", m["tramo"], "", "", "", "", "", "", "", "",
                            "", "", f"{m['rms']:.3f}", f"{m['max']:.3f}", f"{m['atraso']:+.3f}"])

    print(f"{'POS_KI':>14s} | {'reposo: desvío [µm] por repetición':38s} | {'caza [Hz]':16s} | {'asentado <0,05 µm [s]':24s} | "
          f"{'RMS mov. [µm]':14s} | {'máx [µm]':10s} | atraso [µm]")
    for meta, d, rests, moves in rows:
        desv = " ".join(f"{r['desvio']:.3f}" for r in rests)
        caza = " ".join(f"{r['caza_hz']:.1f}" for r in rests)
        asen = " ".join(f"{r['t_asentado']:.2f}" for r in rests)
        rms = np.mean([m["rms"] for m in moves])
        mx = np.max([m["max"] for m in moves])
        atr = np.mean([abs(m["atraso"]) for m in moves])
        print(f"{name(meta).replace(chr(10), ' '):>14s} | {desv:38s} | {caza:16s} | {asen:24s} | "
              f"{rms:14.2f} | {mx:10.1f} | {atr:.2f}")

    # --- figura principal ---
    xs = np.arange(len(rows))
    labels = [name(r[0]) for r in rows]
    fig, axs = plt.subplots(1, 3, figsize=(17, 5.2), gridspec_kw=dict(wspace=0.28))
    a = axs[0]
    for i, (meta, d, rests, moves) in enumerate(rows):
        v = [r["desvio"] for r in rests]
        a.plot([i] * len(v), v, "o", color=T.DIR_COLOR[+1], alpha=0.6, ms=7)
        a.plot([i - 0.25, i + 0.25], [np.median(v)] * 2, color=T.INK, lw=2)
        a.annotate(f"{np.median(v):.3f}", (i + 0.27, np.median(v)), fontsize=8, va="center", color=T.INK2)
    a.set_title("(a) Quieto: desvío del error en cada reposo", loc="left")
    a.set_ylabel("µm")
    a.set_yscale("log")
    a = axs[1]
    for i, (meta, d, rests, moves) in enumerate(rows):
        v = [abs(r["e_0.5"]) for r in rests]
        a.plot([i] * len(v), np.maximum(v, 1e-3), "s", color=T.DIR_COLOR[-1], alpha=0.6, ms=7)
        a.plot([i - 0.25, i + 0.25], [max(np.median(v), 1e-3)] * 2, color=T.INK, lw=2)
    a.set_title("(b) Llegada: |error| 0,5 s después de parar", loc="left")
    a.set_ylabel("µm")
    a.set_yscale("log")
    a = axs[2]
    for i, (meta, d, rests, moves) in enumerate(rows):
        a.plot([i] * len(moves), [m["rms"] for m in moves], "o", color=T.DIR_COLOR[+1], alpha=0.6, ms=7)
        a.plot([i] * len(moves), [m["max"] for m in moves], "s", color=T.DIR_COLOR[-1], alpha=0.6, ms=7)
    a.plot([], [], "o", color=T.DIR_COLOR[+1], label="RMS de cada tramo")
    a.plot([], [], "s", color=T.DIR_COLOR[-1], label="máximo de cada tramo")
    a.legend(loc="center right", fontsize=8)
    a.set_title("(c) En movimiento (2 mm/s)", loc="left")
    a.set_ylabel("µm")
    a.set_ylim(0, None)
    for a in axs:
        a.set_xticks(xs)
        a.set_xticklabels(labels, fontsize=8)
        a.set_xlabel("POS_KI")
    end = rows[0][0]["end_mm"]
    fig.suptitle(f"Elección de POS_KI: cada punto es una repetición (reposos en X = 0 y X = {end:g} mm; "
                 f"línea = mediana)", x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.05, right=0.98, top=0.84, bottom=0.17)
    p1 = os.path.join(out_dir, "eleccion_pos_ki.png")
    fig.savefig(p1, dpi=110)
    plt.close(fig)

    # --- trazas de reposo ---
    ncol = max(len(r[2]) for r in rows)
    fig, axs = plt.subplots(len(rows), ncol, figsize=(4 * ncol, 1.45 * len(rows) + 1.3), sharex=True, sharey=True,
                            squeeze=False, gridspec_kw=dict(hspace=0.55, wspace=0.06))
    lim = 0.0
    for meta, d, rests, moves in rows:
        for r in rests:
            lim = max(lim, np.max(np.abs(d["err_um"][r["w"]][d["t_s"][r["w"]] - d["t_s"][r["stop"]] >= 0.2])))
    lim = min(lim * 1.1, 2.0)
    for i, (meta, d, rests, moves) in enumerate(rows):
        for j in range(ncol):
            a = axs[i, j]
            if j >= len(rests):
                a.set_visible(False)
                continue
            r = rests[j]
            tt = d["t_s"][r["w"]] - d["t_s"][r["stop"]]
            a.plot(tt, d["err_um"][r["w"]], color=T.DIR_COLOR[+1], lw=0.8)
            a.axvspan(0, SKIP_S, color=T.MUTED, alpha=0.1, lw=0)
            a.axhline(0, color=T.AXIS, lw=0.8)
            a.set_ylim(-lim, lim)
            a.set_title(f"POS_KI {name(meta).replace(chr(10), ' ')} · X={r['x_mm']:g} · desvío {r['desvio']:.3f} µm",
                        fontsize=8, loc="left")
            if j == 0:
                a.set_ylabel("µm", fontsize=8)
    for j in range(ncol):
        axs[-1, j].set_xlabel("segundos desde que el perfil se detiene")
    fig.suptitle("Error en cada reposo (sombreado: primer segundo, llegada; no entra en el desvío). Misma escala en todos",
                 x=0.01, ha="left", fontsize=11, fontweight="bold")
    h = 1.45 * len(rows) + 1.3
    fig.subplots_adjust(left=0.04, right=0.99, top=1 - 0.7 / h, bottom=0.5 / h)
    p2 = os.path.join(out_dir, "reposo_trazas.png")
    fig.savefig(p2, dpi=95)
    plt.close(fig)
    print("\n  " + p1 + "\n  " + p2)


if __name__ == "__main__":
    main()
