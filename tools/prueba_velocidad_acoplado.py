#!/usr/bin/env python3
"""Prueba de velocidad con el carro acoplado y control de la conmutacion entre velocidades.

Para cada velocidad: ida y vuelta 0 -> --end -> 0 con telemetria (tools/closed_loop_test.py) y
despues M57 X5 desde 0 mm. M57 da el desfase campo - tabla en lazo abierto (incluye el atraso
del rotor por friccion); medido siempre en el mismo tramo y a la misma velocidad, si su media
cambia entre controles es que el encoder se corrio respecto del rotor (rueda floja).

    python3 tools/prueba_velocidad_acoplado.py                       # 2, 3, 4 y 5 mm/s, 0 -> 50 -> 0
    python3 tools/prueba_velocidad_acoplado.py --velocidades 2 2.5 3 --end 30

Requiere el eje homeado y calibrado (M56) y el carro en 0. Resultados en mediciones/AAAA-MM-DD/:
los de closed_loop_test.py (tag acoplado_v<v>) y velocidad_acoplado_HHMMSS.log con los M57.
"""
import argparse
import datetime
import os
import statistics
import subprocess
import sys
import time

import serial

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.dirname(os.path.abspath(__file__))


class Placa:
    def __init__(self, port, log):
        self.port = port
        self.log = log
        self.s = None

    def abrir(self):
        self.s = serial.Serial(self.port, 921600, timeout=0.2)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def cerrar(self):
        if self.s:
            self.s.close()
            self.s = None

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
            if l == "ok" or l.startswith("error"):
                break
        self.log.flush()
        return lineas

    def esperar_quieto(self, destino_mm, timeout=120):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for l in self.cmd("M114", 3):
                if "X:" in l or "pos" in l:
                    try:
                        num = float(l.split("X:")[1].split()[0]) if "X:" in l else None
                    except (IndexError, ValueError):
                        num = None
                    if num is not None and abs(num - destino_mm) < 0.002:
                        return True
            time.sleep(0.5)
        return False


def control_conmutacion(p):
    """M57 X5 desde 0 mm. Devuelve (media, desvio) del desfase en grados electricos."""
    r = p.cmd("M57 X5 F1.6 P4", 60)
    vals = []
    for l in r:
        partes = l.split(",")
        if len(partes) == 4:
            try:
                vals.append(float(partes[2]))
            except ValueError:
                pass
    p.cmd("M17", 5)
    p.cmd("G1 X0 F2", 5)
    p.esperar_quieto(0.0)
    if not vals:
        return float("nan"), float("nan")
    return statistics.fmean(vals), statistics.pstdev(vals)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--velocidades", type=float, nargs="+", default=[2, 3, 4, 5])
    ap.add_argument("--end", type=float, default=50.0)
    args = ap.parse_args()

    ahora = datetime.datetime.now()
    carpeta = os.path.join(ROOT, "mediciones", ahora.strftime("%Y-%m-%d"))
    os.makedirs(carpeta, exist_ok=True)
    ruta = os.path.join(carpeta, ahora.strftime("velocidad_acoplado_%H%M%S.log"))
    filas = []
    with open(ruta, "w") as log:
        p = Placa(args.port, log)
        p.abrir()
        m0, s0 = control_conmutacion(p)
        print(f"referencia M57: desfase medio {m0:+.1f} deg el (desvio {s0:.1f})")
        p.cerrar()

        for v in args.velocidades:
            print(f"\n=== {v:g} mm/s: 0 -> {args.end:g} -> 0 ===", flush=True)
            r = subprocess.run([sys.executable, os.path.join(TOOLS, "closed_loop_test.py"),
                                "--cycles", "1", "--end", f"{args.end:g}", "--feed", f"{v:g}",
                                "--pre", f"M203 X{v:g}", "--tag", f"acoplado_v{v:g}"],
                               capture_output=True, text=True)
            salida = r.stdout + r.stderr
            log.write(f"--- closed_loop_test {v:g} mm/s ---\n{salida}\n")
            ok = "sin cortes" in salida.lower()
            resumen = [l for l in salida.splitlines() if "CORTE POR FALLA" in l or l.startswith("Resultado")]
            print("\n".join(resumen[-3:]))

            p.abrir()
            if not ok:
                p.cmd("M17", 5)          # rearma donde quedo
                p.cmd("G1 X0 F2", 5)
                p.esperar_quieto(0.0, 200)
            m, s = control_conmutacion(p)
            p.cerrar()
            print(f"M57 despues de {v:g} mm/s: desfase medio {m:+.1f} deg el (desvio {s:.1f}), "
                  f"cambio {m - m0:+.1f} deg el respecto de la referencia")
            filas.append((v, ok, m, s, m - m0))

    print(f"\nlog en {ruta}\n")
    print("velocidad  resultado   desfase M57  cambio")
    for v, ok, m, s, c in filas:
        print(f"{v:6.1f}     {'sin cortes' if ok else 'CORTE':10s}  {m:+7.1f}      {c:+6.1f} deg el")


if __name__ == "__main__":
    main()
