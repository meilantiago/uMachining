#!/usr/bin/env python3
"""Prueba de fijacion de la rueda de imanes (motor sin acople).

Mide cuanto se corre la conmutacion despues de rampas fuertes a lazo abierto (M63). Si la rueda
patina sobre el eje, el encoder queda corrido respecto del rotor y G28 lo ve como desfase de
conmutacion: 1 grado de eje = 50 grados electricos.

    python3 tools/prueba_rueda.py                 # serie por defecto
    python3 tools/prueba_rueda.py --repeticiones 2

Secuencia: M56, G28 de referencia y, para cada rampa, M63 + G28. Si G28 rechaza el desfase,
recalibra (M56) y toma una referencia nueva. Requiere el firmware de prueba/sin-acople (M63).
Guarda el log en mediciones/AAAA-MM-DD/prueba_rueda_HHMMSS.log.
"""
import argparse
import datetime
import os
import re
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# (V mm/s con signo, A mm/s2, T s de crucero)
SERIE = [(10, 300, 0.5), (-10, 300, 0.5), (20, 300, 0.5), (-20, 300, 0.5),
         (30, 1000, 0.3), (-30, 1000, 0.3), (40, 300, 0.3), (-40, 300, 0.3)]

RE_DESFASE = re.compile(r"Conmutacion: desfase (-?[\d.]+) deg electricos \(con el encoder corrido -1 periodo: "
                        r"(-?[\d.]+), \+1: (-?[\d.]+)\), atraso del rotor (-?[\d.]+) deg(?:, consistencia (-?[\d.]+))?")
# la placa corta las lineas largas del log: la consistencia puede no llegar
RE_M63 = re.compile(r"lazo_abierto: .*")


class Placa:
    def __init__(self, port, log):
        self.s = serial.Serial(port, 921600, timeout=0.2)
        self.log = log
        time.sleep(0.5)
        self.s.reset_input_buffer()

    def cmd(self, linea, timeout):
        self.log.write(f"> {linea}\n")
        self.s.write((linea + "\n").encode())
        lineas = []
        t0 = time.time()
        while time.time() - t0 < timeout:
            l = self.s.readline().decode(errors="replace").strip()
            if not l:
                continue
            self.log.write(l + "\n")
            lineas.append(l)
            if l == "ok" or l.startswith("ok ") or l.startswith("error"):
                break
        self.log.flush()
        return lineas


def desfase(lineas):
    """Desfase (como esta, -1 periodo, +1 periodo, el menor en modulo, consistencia) o None."""
    for l in lineas:
        m = RE_DESFASE.search(l)
        if m:
            d0, dm, dp = (float(x) for x in m.groups()[:3])
            cons = float(m.group(5)) if m.group(5) else float("nan")
            menor = min((d0, dm, dp), key=abs)
            return d0, menor, cons
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--repeticiones", type=int, default=1)
    args = ap.parse_args()

    ahora = datetime.datetime.now()
    carpeta = os.path.join(ROOT, "mediciones", ahora.strftime("%Y-%m-%d"))
    os.makedirs(carpeta, exist_ok=True)
    ruta = os.path.join(carpeta, ahora.strftime("prueba_rueda_%H%M%S.log"))
    filas = []
    with open(ruta, "w") as log:
        p = Placa(args.port, log)

        def calibrar():
            r = p.cmd("M56", 120)
            d = desfase(r)
            ok = any(l == "ok" for l in r)
            print(f"M56: {'ok' if ok else 'FALLO'}  desfase al calibrar {d[0] if d else float('nan'):+.1f} deg el")
            r = p.cmd("G28", 30)
            d = desfase(r)
            print(f"G28 de referencia: {d[0] if d else float('nan'):+.1f} deg el (queda {d[1] if d else float('nan'):+.1f})")
            return d[1] if d else float("nan")

        ref = calibrar()
        for rep in range(args.repeticiones):
            for v, a, t in SERIE:
                r = p.cmd(f"M63 V{v} A{a} T{t} P1000", 60)
                res = next((l for l in r if RE_M63.search(l)), "sin resumen")
                perdio = "PERDIO" in res
                r = p.cmd("G28", 30)
                d = desfase(r)
                fallo = not any(l == "ok" for l in r)
                d0 = d[0] if d else float("nan")
                menor = d[1] if d else float("nan")
                cons = d[2] if d else float("nan")
                cambio = menor - ref   # sin el periodo que G28 corrige solo
                fila = (rep + 1, v, a, t, perdio, ref, d0, menor, cambio, cons, fallo)
                filas.append(fila)
                print(f"[{rep + 1}] M63 V{v:+4d} A{a:5d}: {'perdio el paso' if perdio else 'sin perdida':15s} "
                      f"G28 desfase {d0:+7.1f} (menor {menor:+7.1f}) cambio {cambio:+7.1f} deg el "
                      f"= {cambio / 50:+.3f} deg eje  consist {cons:.2f}{'  G28 RECHAZO' if fallo else ''}")
                if fallo:
                    ref = calibrar()
                else:
                    ref = menor   # G28 corrige el contador si quedo corrido un periodo
        p.cmd("M18", 5)

    print(f"\nlog en {ruta}")
    cambios = [abs(f[8]) for f in filas if f[8] == f[8]]
    if cambios:
        print(f"cambio de conmutacion por rampa: mediana {sorted(cambios)[len(cambios) // 2]:.1f}, "
              f"maximo {max(cambios):.1f} deg el (= {max(cambios) / 50:.3f} deg de eje); "
              f"G28 rechazo {sum(f[10] for f in filas)} de {len(filas)}")


if __name__ == "__main__":
    main()
