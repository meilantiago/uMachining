#!/usr/bin/env python3
"""Consolida las escaleras de 10 nm del 06/11/2026 (o las que se pasen) en una tabla.

    python3 informe/scripts/resumen_escaleras.py mediciones/2026-11-06/escalera_*.csv

Por corrida: desvio en reposo (sin el primer segundo), salto y desvio de cada escalon
(sin el primer segundo de cada uno), mismo criterio que tools/escalera_nm.py.
Al final: estadistica de todos los saltos juntos.
"""
import sys
import numpy as np


def leer(ruta):
    with open(ruta) as f:
        cab = f.readline().lstrip("# ").strip()
    d = np.genfromtxt(ruta, delimiter=",", skip_header=2)
    d = d[d[:, 1] >= 0]
    return cab, d[:, 0] - d[0, 0], d[:, 1].astype(int), d[:, 3], d[:, 4]


def main():
    saltos, desvios_esc, reposos = [], [], []
    print("| corrida | reposo σ [nm] | reposo pp [nm] | saltos [nm] | σ por escalón [nm] |")
    print("|---|---|---|---|---|")
    for ruta in sys.argv[1:]:
        cab, t, tramo, pos, err = leer(ruta)
        niv = []
        for k in range(tramo.max() + 1):
            sel = tramo == k
            if not sel.any():
                continue
            m = sel & (t > t[sel][0] + 1.0)
            niv.append((k, pos[m].mean(), pos[m].std(), np.ptp(pos[m])))
        rep = niv[0]
        s = [niv[i][1] - niv[i - 1][1] for i in range(1, len(niv))]
        de = [n[2] for n in niv[1:]]
        saltos += s
        desvios_esc += de
        reposos.append(rep[2])
        nombre = ruta.split("/")[-1].replace(".csv", "")
        print(f"| {nombre} | {rep[2]:.2f} | {rep[3]:.1f} | "
              + ", ".join(f"{x:+.1f}" for x in s) + " | "
              + ", ".join(f"{x:.1f}" for x in de) + " |")
    s = np.array(saltos)
    de = np.array(desvios_esc)
    print()
    print(f"corridas: {len(reposos)}; reposo σ: mediana {np.median(reposos):.2f} nm, rango {min(reposos):.2f}-{max(reposos):.2f} nm")
    print(f"saltos: N={len(s)}, media {s.mean():+.2f} nm, desvio {s.std(ddof=1):.2f} nm, "
          f"mediana {np.median(s):+.2f} nm, rango {s.min():+.1f} a {s.max():+.1f} nm")
    lim = 5.0
    ok = de < lim
    print(f"escalones con σ < {lim} nm: {ok.sum()}/{len(de)}; saltos de esos: media {s[ok].mean():+.2f} nm, desvio {s[ok].std(ddof=1):.2f} nm")


if __name__ == "__main__":
    main()
