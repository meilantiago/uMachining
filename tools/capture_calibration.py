#!/usr/bin/env python3
"""Envia 'M56 P' al firmware, guarda todas las muestras de la calibracion en CSV,
imprime un analisis y genera un grafico PNG.

Uso:
    python3 tools/capture_calibration.py [--port /dev/ttyACM0] [--out calib]

Cerrar antes cualquier otro programa que tenga el puerto abierto (Teleplot,
pio device monitor, etc): Linux solo deja un proceso por puerto.
"""
import argparse
import time

import numpy as np
import serial
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

MT6835_CPR = 1 << 21
RAD2DEG = 180.0 / np.pi


def capture(port, baud, command, timeout_s):
    lines = []
    with serial.Serial(port, baud, timeout=0.5) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write((command + "\n").encode())
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").rstrip("\r\n")
            lines.append(line)
            if line.startswith("ok") or line.startswith("error"):
                break
    return lines


def parse(lines):
    rows = []  # motor_pos_rad, field_angle_rad, encoder_raw, weak_field
    for line in lines:
        if line.startswith(">weak_field:") and rows:
            rows[-1][3] = int(float(line.split(":")[1]))
            continue
        parts = line.split(",")
        if len(parts) != 3:
            continue
        try:
            rows.append([float(parts[0]), float(parts[1]), float(parts[2]), 0])
        except ValueError:
            pass
    return np.array(rows) if rows else np.zeros((0, 4))


def analyze_pass(name, mp, enc):
    coef = np.polyfit(enc, mp, 1)
    resid = mp - np.polyval(coef, enc)
    dm = np.diff(mp)
    de = np.diff(enc)
    direction = np.sign(np.sum(dm))
    enc_dir = np.sign(np.sum(de))
    backward_steps = int(np.sum(de * enc_dir < 0))
    counts_per_motor_rad = 1.0 / coef[0]
    periods_per_rev = abs(counts_per_motor_rad) * 2 * np.pi / MT6835_CPR
    print(f"--- pasada {name}: {len(mp)} muestras")
    print(f"  motor_pos: {mp[0]*RAD2DEG:.2f} -> {mp[-1]*RAD2DEG:.2f} deg (sentido {direction:+.0f})")
    print(f"  encoder:   {enc[0]:.0f} -> {enc[-1]:.0f} counts (delta {enc[-1]-enc[0]:.0f})")
    print(f"  pendiente: {counts_per_motor_rad:.0f} counts/rad de motor -> {periods_per_rev:.2f} periodos de encoder por vuelta")
    print(f"  residuo vs recta: rms {np.sqrt(np.mean(resid**2))*RAD2DEG:.3f} deg, max {np.max(np.abs(resid))*RAD2DEG:.3f} deg")
    print(f"  pasos donde el encoder va hacia atras: {backward_steps} de {len(de)}")
    return coef, resid


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--command", default="M56 P")
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--out", default="calib")
    ap.add_argument("--from-log", help="analizar un log ya capturado en vez de mandar el comando")
    args = ap.parse_args()

    if args.from_log:
        with open(args.from_log) as f:
            lines = f.read().splitlines()
    else:
        lines = capture(args.port, args.baud, args.command, args.timeout)
        with open(args.out + "_log.txt", "w") as f:
            f.write("\n".join(lines) + "\n")

    for line in lines:
        if line[:2] in ("I)", "D)", "W)", "E)") or line.startswith(("ok", "error")):
            print(line)

    data = parse(lines)
    if len(data) < 10:
        print("No se encontraron muestras (se mando con P?).")
        return

    np.savetxt(args.out + ".csv", data, delimiter=",", fmt=["%.10f", "%.10f", "%.1f", "%d"],
               header="motor_pos_rad,field_angle_rad,encoder_raw,weak_field", comments="")
    print(f"{len(data)} muestras guardadas en {args.out}.csv")

    mp, enc, weak = data[:, 0], data[:, 2], data[:, 3]
    imax = int(np.argmax(mp))
    passes = [("ida", slice(0, imax + 1)), ("vuelta", slice(imax, None))]

    fig, ax = plt.subplots(4, 1, figsize=(11, 14))
    for name, s in passes:
        if len(mp[s]) < 10:
            continue
        coef, resid = analyze_pass(name, mp[s], enc[s])
        ax[0].plot(mp[s] * RAD2DEG, enc[s], ".", ms=2, label=name)
        ax[1].plot(mp[s] * RAD2DEG, resid * RAD2DEG, ".", ms=2, label=name)
        phase = np.mod(enc[s], MT6835_CPR) / MT6835_CPR * 360.0
        ax[2].plot(phase, resid * RAD2DEG, ".", ms=2, label=name)
    print(f"weak_field en {int(weak.sum())} de {len(weak)} muestras")

    ax[0].set_xlabel("motor_pos comandado [deg]"); ax[0].set_ylabel("encoder raw [counts]")
    ax[1].set_xlabel("motor_pos comandado [deg]"); ax[1].set_ylabel("residuo vs recta [deg]")
    ax[2].set_xlabel("fase dentro del periodo del encoder [deg]"); ax[2].set_ylabel("residuo [deg]")
    ax[3].plot(np.diff(enc), lw=0.6); ax[3].set_xlabel("muestra"); ax[3].set_ylabel("delta encoder por muestra")
    ax[3].plot(np.where(weak[1:] > 0)[0], np.zeros(int(weak[1:].sum())), "r|", ms=10, label="weak_field")
    for a in ax:
        a.grid(True, alpha=0.3); a.legend()
    fig.tight_layout()
    fig.savefig(args.out + ".png", dpi=90)
    print(f"grafico en {args.out}.png")


if __name__ == "__main__":
    main()
