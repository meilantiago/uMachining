#!/usr/bin/env python3
"""Prueba de torque con el motor sin acople (rama prueba/sin-acople, comando M61).

Con el campo adelantado una distancia fija respecto del rotor (sin PID), el rotor suelto acelera con
toda la fuerza que da ese adelanto. La aceleracion en funcion de la velocidad es la curva de fuerza
disponible del motor (dividida por la inercia, que no conocemos). Frena con el adelanto opuesto.

Uso:
  python3 tools/prueba_torque.py                     # serie completa (ver SERIE)
  python3 tools/prueba_torque.py --analizar DIR      # solo rehace el analisis de una carpeta
"""
import argparse
import datetime
import glob
import math
import os
import re
import sys
import time

import numpy as np

PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PITCH_MM = 5.0
MM_TO_RAD = 2 * math.pi / PITCH_MM

# (adelanto um, velocidad tope mm/s, tiempo tope s). Primero la escalera de velocidad con el adelanto
# del firmware (22,5 um), en los dos sentidos; despues otros adelantos a la misma velocidad.
SERIES = {}
SERIES["adelanto"] = [(None, L, V, T) for L, V, T in [
    (22.5, 10, 0.5), (-22.5, 10, 0.5),
    (22.5, 20, 0.5), (-22.5, 20, 0.5),
    (22.5, 50, 0.5), (-22.5, 50, 0.5),
    (10, 50, 0.5), (15, 50, 0.5), (30, 50, 0.5), (40, 50, 0.5),
    (-10, 50, 0.5), (-15, 50, 0.5), (-30, 50, 0.5), (-40, 50, 0.5),
]]
# Compensacion del atraso de la corriente (M62 T<ms>): con tau bien elegida la velocidad final sube.
# Con tau = 3 ms, subir la tension solo durante la prueba (M61 A): si la velocidad final sube, el techo es tension.
SERIES["tension"] = [("M62 T3", L, 50, 0.6, A) for A in (0.6, 0.8, 1.0) for L in (22.5, -22.5)]
SERIES["compensacion"] = [(f"M62 T{tau:g}", L, 50, 0.6) for tau in (0, 1, 2, 3, 4) for L in (22.5, -22.5)]


class Placa:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 921600, timeout=0.2)
        time.sleep(0.3)
        self.s.reset_input_buffer()

    def comando(self, texto, timeout=10.0):
        """Manda una linea y junta lo que vuelve hasta 'ok' o 'error'."""
        self.s.write((texto + "\n").encode())
        lineas, buf, fin = [], b"", time.monotonic() + timeout
        while time.monotonic() < fin:
            buf += self.s.read(65536)
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                linea = linea.decode(errors="replace").strip()
                if not linea:
                    continue
                lineas.append(linea)
                if linea.startswith("ok") or linea.startswith("error"):
                    return lineas
        raise TimeoutError(f"{texto}: sin respuesta en {timeout} s ({len(lineas)} lineas)")


def correr(args):
    carpeta = os.path.join(REPO, "mediciones", datetime.date.today().isoformat(),
                           "torque_" + datetime.datetime.now().strftime("%H%M%S"))
    os.makedirs(carpeta)
    p = Placa(args.port)
    p.comando("M120 S0")
    time.sleep(0.2)
    p.s.reset_input_buffer()
    estado = p.comando("M119")
    print("\n".join(estado))
    if True:   # siempre: la tabla de la flash puede ser de otro montaje, y sin M56 no hay cero
        print("Calibrando (M56)...")
        r = p.comando("M56", timeout=240)
        print("\n".join(l for l in r if "Conmutacion" in l or "error" in l or l.startswith("ok")))
        with open(os.path.join(carpeta, "M56.txt"), "w") as f:
            f.write("\n".join(r) + "\n")
        if not r[-1].startswith("ok"):
            sys.exit("la calibracion fallo")

    for previo, lead, vmax, tmax, *amp in SERIES[args.serie]:
        if previo:
            print(" ".join(l for l in p.comando(previo) if not l.startswith("ok")))
        texto = f"M61 L{lead:g} V{vmax:g} T{tmax:g}" + (f" A{amp[0]:g}" if amp else "")
        r = p.comando(texto, timeout=30)
        p.comando("M18")   # sin lazo entre pruebas: con las ganancias de siempre oscila
        resumen = [l for l in r if l.startswith("torque:") or l.startswith("error")]
        print(f"{texto:28s} {' | '.join(resumen)}")
        csv = [l for l in r if re.match(r"^\d+,", l)]
        nombre = (f"{previo.replace(' ', '')}_" if previo else "") + f"L{lead:+05.1f}_V{vmax:g}" + (f"_A{amp[0]:g}" if amp else "") + ".csv"
        nombre = nombre.replace("+", "p").replace("-", "m")
        with open(os.path.join(carpeta, nombre), "w") as f:
            f.write(f"# {previo or ''} {texto}\n# {' | '.join(resumen)}\nt_us,pos_mm,fase\n" + "\n".join(csv) + "\n")
        time.sleep(0.5)
    p.comando("M18")
    print(f"\nDatos en {carpeta}")
    return carpeta


def derivar(t, x, k):
    """Derivada centrada con +-k muestras."""
    d = np.full_like(x, np.nan)
    d[k:-k] = (x[2 * k:] - x[:-2 * k]) / (t[2 * k:] - t[:-2 * k])
    return d


def analizar(carpeta):
    filas = []
    curvas = []
    for path in sorted(glob.glob(os.path.join(carpeta, "*L*_V*.csv")), key=os.path.getmtime):
        with open(path) as f:
            cab = f.readline().strip("# \n") + " | " + f.readline().strip("# \n")
        d = np.genfromtxt(path, delimiter=",", skip_header=2, names=True)
        if d.size < 50:
            continue
        t = d["t_us"] * 1e-6
        x = d["pos_mm"] - d["pos_mm"][0]
        fase = d["fase"]
        lead = float(re.search(r"M61 L(-?[\d.]+)", cab).group(1))
        tau = re.search(r"M62 T([\d.]+)", cab)
        tau = float(tau.group(1)) if tau else float("nan")
        s = 1.0 if lead > 0 else -1.0
        v = derivar(t, x * s, 11)         # ~1 ms de ventana, en el sentido del adelanto
        acel = fase == 1

        def vel(t0, t1):
            m = acel & (t >= t0) & (t < t1)
            return np.polyfit(t[m], x[m] * s, 1)[0] if m.sum() > 5 else np.nan
        t_fin = t[acel][-1] if acel.any() else 0.0
        llego = re.search(r"acelero ([\d.]+) ms", cab)
        llego = float(llego.group(1)) if llego else np.nan
        amp = re.search(r"M61 [^|]* A([\d.]+)", cab)
        tau = (tau, float(amp.group(1)) if amp else 0.6)
        filas.append((tau, lead, vel(0.0, 0.002), vel(0.004, 0.006), vel(0.018, 0.022),
                      vel(max(0.0, t_fin - 0.1), t_fin), llego, t_fin * 1e3, os.path.basename(path)))
        curvas.append((tau, lead, t[acel], v[acel]))  # tau = (tau ms, amplitud)

    print(f"\n{'tau ms':>6} {'amp':>4} {'L um':>6} {'v0':>7} {'v 5ms':>7} {'v 20ms':>7} {'v final':>8} {'acelero':>8}   [mm/s, en el sentido de L]")
    for tau, lead, v0, v5, v20, vf, llego, t_fin, nombre in filas:
        tope = "  llego al tope" if np.isfinite(llego) and llego < t_fin - 1 else ""
        print(f"{tau[0]:6.1f} {tau[1]:4.1f} {lead:6.1f} {v0:7.2f} {v5:7.2f} {v20:7.2f} {vf:8.2f} {llego:7.1f}ms{tope}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    for tau, lead, t, v in curvas:
        ax = ax1 if lead > 0 else ax2
        ax.plot(t * 1e3, v, lw=1, label=f"tau {tau[0]:g} ms, amp {tau[1]:g}")
    for ax, sentido in ((ax1, "adelanto +"), (ax2, "adelanto -")):
        ax.set_xlabel("tiempo [ms]"); ax.set_ylabel(f"velocidad en el sentido del adelanto [mm/s] ({sentido})")
        ax.grid(alpha=.3); ax.legend(fontsize=8)
    fig.tight_layout()
    out = os.path.join(carpeta, "torque.png")
    fig.savefig(out, dpi=110)
    print(f"Grafico: {out}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--analizar", metavar="CARPETA")
    ap.add_argument("--serie", choices=sorted(SERIES), default="compensacion")
    args = ap.parse_args()
    analizar(args.analizar or correr(args))
