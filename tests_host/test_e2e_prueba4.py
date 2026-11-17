#!/usr/bin/env python3
"""Test de punta a punta del M60.

1. La consigna sale del código del firmware: test_senoide --csv corre un barrido de 13
   M60 (1 a 200 Hz) por el CommandParser real y la composición trapecio + senoide real.
2. Esa consigna entra a la máquina simulada de la prueba 4 (lazo del firmware, mecánica
   con fricción, Michelson en cuadratura, relojes distintos).
3. El análisis de la prueba 4 encuentra cada tramo SOLO con la marca de medición
   (la que en la placa sale por PIN_SINE_SYNC y por la telemetría, flags bit2), sin
   conocer el cronograma, y calcula el Bode.

Pasa si se encuentran los 13 tramos con la frecuencia y la duración pedidas y si el Bode
estimado coincide con el verdadero.
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

RAIZ = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RAIZ / "tools"))
sys.path.insert(0, str(RAIZ / "simulaciones"))

import prueba2_analisis as p2  # noqa: E402
import prueba4_analisis as an  # noqa: E402
import prueba4_simulacion as sim  # noqa: E402

FRECUENCIAS = [1, 2, 5, 10, 20, 30, 50, 70, 100, 130, 150, 170, 200]
X0_MM = 30.0


def tramos_por_marca(t, marca, f_hz):
    """Ventanas de medición a partir de la marca (flancos) y la frecuencia registrada."""
    m = marca.astype(int)
    sube = np.flatnonzero(np.diff(m) == 1) + 1
    baja = np.flatnonzero(np.diff(m) == -1) + 1
    return [dict(f=float(f_hz[a]), t0=float(t[a]), t1=float(t[b])) for a, b in zip(sube, baja)]


def main(csv: str) -> int:
    fallas = []
    d = np.genfromtxt(csv, delimiter=",", names=True)
    paso = np.diff(d["t_s"])
    if np.max(np.abs(paso - 5e-4)) > 1e-9:
        fallas.append("la consigna no está a 2 kHz exactos")

    # 2. máquina simulada (desvíos respecto de X0)
    dyn = sim.simular_dinamica(X0_MM, d["pos_mm"] - X0_MM, d["vel_mm_s"], seed=7)
    q = sim.FS_SERVO // sim.FS_TELEM                     # 16 ticks de servo por punto
    r = sim.FS_SERVO // sim.FS_PERFIL                    # 8 ticks de servo por punto del perfil
    tel = dict(t=dyn["t"][::q], xc=dyn["xc"][::q], xe=dyn["enc"][::q])
    marca = d["midiendo"][::q // r]                      # la marca viaja con la telemetría
    f_reg = d["f_hz"][::q // r]
    n = min(len(tel["t"]), len(marca))
    tel = {k: v[:n] for k, v in tel.items()}
    marca, f_reg = marca[:n], f_reg[:n]

    # 3. tramos solo con la marca
    segs = tramos_por_marca(tel["t"], marca, f_reg)
    fs_encontradas = [s["f"] for s in segs]
    if fs_encontradas != FRECUENCIAS:
        fallas.append(f"tramos encontrados {fs_encontradas}")
    for s in segs:
        n_ciclos = int(np.ceil(max(5.0, 0.5 * s["f"])))
        dur = s["t1"] - s["t0"]
        if abs(dur - n_ciclos / s["f"]) > 1.5e-3:
            fallas.append(f"{s['f']} Hz: ventana de {dur:.4f} s, esperada {n_ciclos / s['f']:.4f} s")

    # Michelson + demodulación, con sincronismo por hardware (el pin de la marca)
    t_i, I1, I2, _ = sim.michelson(dyn["t"], dyn["x"], X0_MM)
    fase, _ = p2.fase_cuadratura(I1, I2)
    x_i = p2.fase_a_mm(fase)
    x_i -= x_i[0]
    t_id, x_id = p2.diezmar(t_i, x_i, 5000.0)
    est = an.analizar_barrido(segs, tel["t"], tel["xc"], tel["xe"], t_id, x_id,
                              sim.RELOJ_DESFASE_S, sim.RELOJ_PPM)
    ver = sim.verdad_barrido(segs, tel, dyn)

    sys.stdout.flush()
    print("| f (Hz) | A comandada (nm) | A carro (nm) | |G_i| est. (dB) | verdad | ∠G_i est. (°) | verdad |")
    print("|---|---|---|---|---|---|---|")
    for e, v in zip(est, ver):
        dg = an.db(e["Gi"]) - an.db(v["Gi"])
        dp = an.fase_grad(e["Gi"] / v["Gi"])
        print(f"| {e['f']:g} | {e['A_c_nm']:.0f} | {e['A_i_nm']:.0f} | {an.db(e['Gi']):+.3f} | "
              f"{an.db(v['Gi']):+.3f} | {an.fase_grad(e['Gi']):+.2f} | {an.fase_grad(v['Gi']):+.2f} |")
        if abs(e["A_c_nm"] - 1000) > 1:
            fallas.append(f"{e['f']} Hz: amplitud comandada {e['A_c_nm']:.2f} nm")
        if abs(dg) > 0.05:
            fallas.append(f"{e['f']} Hz: ganancia {dg:+.3f} dB respecto de la verdad")
        if abs(dp) > 0.5:
            fallas.append(f"{e['f']} Hz: fase {dp:+.2f}° respecto de la verdad")
        if e["A_i_nm"] < 50:
            fallas.append(f"{e['f']} Hz: el carro casi no se movió ({e['A_i_nm']:.1f} nm)")

    for f in fallas:
        print("  FALLA [e2e]", f)
    print(f"punta a punta: {len(segs)} tramos, {len(fallas)} fallas")
    return 0 if not fallas else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
