#!/usr/bin/env python3
"""Compila el firmware y copia el .uf2 al paquete de Python (umachining/firmware/).

Para quien mantiene el repo: correrlo después de cambiar algo en firmware/src/ y antes de
commitear, así `pip install` instala el firmware nuevo. Actualizar a mano umachining/firmware/INFO.txt.

    python firmware/empaquetar.py
"""
import hashlib
import os
import shutil
import subprocess
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
DESTINO = os.path.join(os.path.dirname(AQUI), "umachining", "firmware")


def pio():
    for c in (shutil.which("pio"), shutil.which("platformio"),
              os.path.expanduser("~/.platformio/penv/bin/pio"),
              os.path.expanduser("~/.platformio/penv/Scripts/pio.exe")):
        if c and os.path.exists(c):
            return c
    sys.exit("No se encontró PlatformIO (pio). Instalarlo con: pip install platformio")


def main():
    subprocess.run([pio(), "run", "-e", "pico"], cwd=AQUI, check=True)
    uf2 = os.path.join(AQUI, ".pio", "build", "pico", "firmware.uf2")
    shutil.copyfile(uf2, os.path.join(DESTINO, "firmware.uf2"))
    with open(uf2, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    with open(os.path.join(DESTINO, "SHA256SUMS"), "w") as f:
        f.write(f"{sha}  firmware.uf2\n")
    print(f"\nfirmware.uf2 copiado a {DESTINO}\nSHA-256 {sha}\nActualizar INFO.txt y commitear.")


if __name__ == "__main__":
    main()
