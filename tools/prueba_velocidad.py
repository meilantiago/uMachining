#!/usr/bin/env python3
"""Velocidad maxima con el lazo cerrado sin errores de seguimiento (rama prueba/sin-acople).

Calibra (M56), fija ganancias y compensacion (M55, M62), y hace idas y vueltas con G1 a velocidades
crecientes registrando la telemetria (M120 P2 = 1 kHz). Despues de cada velocidad verifica con G28 que la
conmutacion no se haya corrido (la rueda se corre con las exigencias fuertes); si se corrio, recalibra.
Para al primer corte por falla o cuando el error pasa de --error-max-um.

Uso:
  python3 tools/prueba_velocidad.py [--velocidades 2,5,10,...] [--m55 "..."] [--m62 "..."]
"""
import argparse
import datetime
import os
import re
import struct
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prueba_torque import Placa, PORT, REPO  # noqa: E402

PUNTO = struct.Struct("<IiiffiBBH")   # TelemetryPoint (src/shared_data.h), 28 bytes
CABECERA = b"\xaa\x56"
LSB_MM = 0.25e-6


def leer_telemetria(datos):
    """Saca los puntos de telemetria de un bloque de bytes (el texto queda afuera: es ASCII < 0x80)."""
    pts, i = [], 0
    while True:
        i = datos.find(CABECERA, i)
        if i < 0 or i + 2 + PUNTO.size > len(datos):
            break
        t, tq, pq, vel, out, raw, status, flags, seq = PUNTO.unpack_from(datos, i + 2)
        pts.append((t, tq * LSB_MM, pq * LSB_MM, vel, out, flags))
        i += 2 + PUNTO.size
    return np.array(pts) if pts else np.zeros((0, 6))


def mover(p, destino, v, acel, archivo):
    """G1 a 'destino' registrando telemetria hasta que termina (o salta el corte)."""
    p.s.reset_input_buffer()
    p.s.write(b"M120 S1 P2\n")
    time.sleep(0.1)
    p.s.write(f"G1 X{destino:.6f} F{v:g}\n".encode())
    datos = b""
    fin = time.monotonic() + abs(destino) / v + v / acel + 1.5
    while time.monotonic() < fin:
        datos += p.s.read(65536)
    p.s.write(b"M120 S0\n")
    time.sleep(0.2)
    datos += p.s.read(65536)
    tel = leer_telemetria(datos)
    np.savetxt(archivo, tel, delimiter=",", fmt=["%d", "%.7f", "%.7f", "%.4f", "%.4f", "%d"],
               header="t_us,objetivo_mm,pos_mm,vel_rad_s,out_rad,flags", comments="")
    p.s.reset_input_buffer()
    return tel


def resumen(tel, v):
    if len(tel) < 10:
        return None
    err_um = (tel[:, 1] - tel[:, 2]) * 1000
    vobj = np.gradient(tel[:, 1], tel[:, 0] * 1e-6)
    crucero = np.abs(np.abs(vobj) - v) < 0.02 * v + 0.05
    corte = bool((tel[:, 5].astype(int) & 2).any())
    out_deg = np.degrees(np.abs(tel[:, 4]))
    return {
        "err_max": float(np.max(np.abs(err_um))),
        "err_crucero_rms": float(np.sqrt(np.mean(err_um[crucero] ** 2))) if crucero.sum() > 10 else float("nan"),
        "err_crucero_max": float(np.max(np.abs(err_um[crucero]))) if crucero.sum() > 10 else float("nan"),
        "out_sat": float(np.mean(out_deg > 79) * 100),
        "corte": corte,
        "puntos": len(tel),
    }


def conmutacion(p):
    r = p.comando("G28", timeout=60)
    lin = next((l for l in r if "Conmutacion: desfase" in l), "")
    # G28 corrige el periodo del encoder (+-120 deg): lo que importa es el menor de los tres desfases
    vals = [float(x) for x in re.findall(r"(-?\d+\.\d+)", lin.split("atraso")[0])]
    return (min(vals, key=abs) if vals else float("nan")), r[-1].startswith("ok")


def preparar(p, args, carpeta, n):
    r = p.comando("M56", timeout=240)
    with open(os.path.join(carpeta, f"M56_{n}.txt"), "w") as f:
        f.write("\n".join(r) + "\n")
    print("  " + next((l for l in r if "Conmutacion" in l), r[-1])[:110])
    if not r[-1].startswith("ok"):
        sys.exit("la calibracion fallo")
    for c in (args.m55, args.m62, f"M204 X{args.acel:g}", "M17"):
        r = p.comando(c)
        print(f"  {c}: {' '.join(l for l in r if not l.startswith('ok'))}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--velocidades", default="2,5,10,15,20,25,30,40,50")
    ap.add_argument("--recorrido", type=float, default=30.0, help="mm de cada ida")
    ap.add_argument("--acel", type=float, default=200.0, help="mm/s2 (M204)")
    ap.add_argument("--m55", default="M55 A60 B100 C0.15 D7.5 F0.001")
    ap.add_argument("--m62", default="M62 T3 K1")
    ap.add_argument("--error-max-um", type=float, default=200.0)
    ap.add_argument("--vuelta", type=float, default=0.0,
                    help="mm/s de las vueltas (0 = la misma velocidad que la ida); las vueltas lentas no se juzgan")
    ap.add_argument("--desfase-max", type=float, default=20.0, help="grados electricos para recalibrar")
    args = ap.parse_args()

    carpeta = os.path.join(REPO, "mediciones", datetime.date.today().isoformat(),
                           "velocidad_" + datetime.datetime.now().strftime("%H%M%S"))
    os.makedirs(carpeta)
    with open(os.path.join(carpeta, "config.txt"), "w") as f:
        f.write(f"{vars(args)}\n")
    p = Placa(args.port)
    p.comando("M120 S0")
    time.sleep(0.2)
    p.s.reset_input_buffer()
    print("Calibrando...")
    preparar(p, args, carpeta, 0)

    # quieto 1 s: si el lazo oscila con estas ganancias no tiene sentido seguir
    tel = mover(p, 0.0, 1.0, args.acel, os.path.join(carpeta, "quieto.csv"))
    out = np.degrees(np.abs(tel[:, 4])) if len(tel) else np.array([90.0])
    err = np.abs(tel[:, 1] - tel[:, 2]) * 1000 if len(tel) else np.array([1e9])
    print(f"Quieto: error max {err.max():.2f} um, salida max {out.max():.1f} deg ({len(tel)} puntos)")
    if out.max() > 60 or err.max() > 20:
        sys.exit("El lazo no queda quieto con estas ganancias: no sigo.")

    print(f"\n{'v mm/s':>7} {'sentido':>7} {'err max':>8} {'crucero rms':>11} {'crucero max':>11} {'out>79°':>8}  [um]")
    recal = 1
    for v in [float(x) for x in args.velocidades.split(",")]:
        r = p.comando(f"M203 X{v:g}")
        malo = False
        for destino, sentido in ((args.recorrido, "+"), (0.0, "-")):
            vv = args.vuelta if (sentido == "-" and args.vuelta > 0) else v
            if vv != v:
                p.comando(f"M203 X{vv:g}")
            tel = mover(p, destino, vv, args.acel, os.path.join(carpeta, f"v{v:g}_{'ida' if sentido == '+' else 'vuelta'}.csv"))
            s = resumen(tel, vv)
            if s is None:
                print(f"{v:7.1f} {sentido:>7}  sin telemetria")
                malo = True
                break
            print(f"{vv:7.1f} {sentido:>7} {s['err_max']:8.1f} {s['err_crucero_rms']:11.2f} {s['err_crucero_max']:11.1f} "
                  f"{s['out_sat']:7.1f}%" + ("  CORTE POR FALLA" if s["corte"] else ""))
            if s["corte"] or s["err_max"] > args.error_max_um:
                malo = True
                break
        desfase, ok = conmutacion(p)
        print(f"        conmutacion despues: {desfase:.1f} deg")
        if malo:
            print(f"Paro en {v:g} mm/s.")
            break
        if not ok or abs(desfase) > args.desfase_max:
            print("        la conmutacion se corrio: recalibro")
            preparar(p, args, carpeta, recal)
            recal += 1
        else:
            p.comando("M17")
        # G28/M56 ponen el cero donde esta el motor: la proxima ida sale de ahi
    p.comando("M18")
    print(f"\nDatos en {carpeta}")


if __name__ == "__main__":
    main()
