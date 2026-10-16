#!/usr/bin/env python3
"""Manda comandos G-code al firmware, uno por uno, e imprime las respuestas.

Uso:
    python3 tools/send_gcode.py "M56" "M17" "G1 X0.2" "wait 3" "M114"

'wait N' espera N segundos sin mandar nada (mientras sigue mostrando lo que llega).
Cerrar antes cualquier otro programa que tenga el puerto abierto.
"""
import argparse
import time

import serial


def read_until_reply(ser, timeout_s):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").rstrip("\r\n")
        if line.startswith(">"):
            continue
        print("  " + line)
        if line.startswith("ok") or line.startswith("error"):
            return


def drain(ser, seconds):
    t0 = time.time()
    while time.time() - t0 < seconds:
        raw = ser.readline()
        if raw:
            line = raw.decode(errors="replace").rstrip("\r\n")
            if not line.startswith(">"):
                print("  " + line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("commands", nargs="+")
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--timeout", type=float, default=120.0)
    args = ap.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.2) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        for cmd in args.commands:
            if cmd.startswith("wait"):
                drain(ser, float(cmd.split()[1]))
                continue
            print("> " + cmd)
            ser.write((cmd + "\n").encode())
            read_until_reply(ser, args.timeout)


if __name__ == "__main__":
    main()
