#!/usr/bin/env python3
"""Reposo y escalera de pasos nanometricos con telemetria a 1 kHz.

Lleva el carro a --pos, graba --reposo s quieto y despues hace --pasos pasos de --paso-nm
(absolutos, G1) con --escalon s en cada uno. Informa el desvio en reposo y el nivel medio de
cada escalon, y guarda un CSV con la telemetria en mediciones/AAAA-MM-DD/escalera_HHMMSS.csv.

    python3 tools/escalera_nm.py                          # 30 mm, 10 s quieto, 5 pasos de 10 nm
    python3 tools/escalera_nm.py --pos 200 --paso-nm 5

Requiere el eje homeado y calibrado (M56). Usa el Link de closed_loop_test.py.
"""
import argparse
import datetime
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import closed_loop_test as T  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--pos", type=float, default=30.0, help="posicion de la prueba [mm]")
    ap.add_argument("--reposo", type=float, default=10.0, help="tiempo quieto antes de los pasos [s]")
    ap.add_argument("--paso-nm", type=float, default=10.0)
    ap.add_argument("--pasos", type=int, default=5)
    ap.add_argument("--escalon", type=float, default=4.0, help="tiempo en cada escalon [s]")
    ap.add_argument("--asentar", type=float, default=5.0, help="espera despues de llegar a --pos [s]")
    args = ap.parse_args()

    link = T.Link(args.port, 921600)
    st = T.parse_status(link.command("M119", echo=False))
    if st.get("homed") != "1" or st.get("calibrated") != "1":
        sys.exit("el eje no esta homeado/calibrado: correr M56 primero")
    link.command("M17", echo=False)
    link.command("G90", echo=False)
    link.command(f"G1 X{args.pos:.6f} F2", echo=False)
    link.wait_text(abs(args.pos) / 2.0 + 3.0 + args.asentar)

    link.command("M120 S1 P2", echo=False)
    link.current_leg = 0
    link.wait_text(args.reposo)
    for k in range(1, args.pasos + 1):
        link.current_leg = k
        link.command(f"G1 X{args.pos + k * args.paso_nm * 1e-6:.7f} F2", echo=False)
        link.wait_text(args.escalon)
    link.command("M120 S0", echo=False)
    link.close()

    d = np.array(link.rows, dtype=float)
    leg = np.array(link.leg)
    t = (d[:, 0] - d[0, 0]) * 1e-6
    t = np.where(t < 0, t + 2**32 * 1e-6, t)            # t_us es de 32 bits
    target, pos = d[:, 1], d[:, 2]
    base = args.pos
    err_nm = (pos - target) * 1e6

    ahora = datetime.datetime.now()
    carpeta = os.path.join(ROOT, "mediciones", ahora.strftime("%Y-%m-%d"))
    os.makedirs(carpeta, exist_ok=True)
    ruta = os.path.join(carpeta, ahora.strftime("escalera_%H%M%S.csv"))
    with open(ruta, "w") as f:
        f.write(f"# escalera: pos {args.pos} mm, {args.pasos} pasos de {args.paso_nm} nm, "
                f"{args.reposo} s quieto, {args.escalon} s por escalon\n")
        f.write("t_s,tramo,objetivo_nm,posicion_nm,error_nm,salida_rad\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.6f},{leg[i]},{(target[i] - base) * 1e6:.3f},{(pos[i] - base) * 1e6:.3f},"
                    f"{err_nm[i]:.3f},{d[i, 4]:.5f}\n")

    m = (leg == 0) & (t > t[leg == 0][0] + 1.0)          # reposo sin el primer segundo
    print(f"reposo a {args.pos:g} mm ({m.sum() / 1000:.1f} s): desvio {np.std(pos[m]) * 1e6:.2f} nm, "
          f"pico a pico {np.ptp(pos[m]) * 1e6:.1f} nm, error medio {np.mean(err_nm[m]):+.2f} nm")
    prev = np.mean(pos[m])
    for k in range(1, args.pasos + 1):
        sel = leg == k
        if not sel.any():
            continue
        ts = t[sel]
        mk = sel & (t > ts[0] + 1.0)                     # descarta el primer segundo de cada escalon
        nivel = np.mean(pos[mk])
        print(f"escalon {k}: objetivo {+k * args.paso_nm:5.1f} nm  nivel {(nivel - base) * 1e6:+7.2f} nm  "
              f"salto {(nivel - prev) * 1e6:+6.2f} nm  desvio {np.std(pos[mk]) * 1e6:.2f} nm")
        prev = nivel
    print(f"datos en {ruta}")


if __name__ == "__main__":
    main()
