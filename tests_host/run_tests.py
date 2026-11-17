#!/usr/bin/env python3
"""Compila y corre los tests de PC del firmware (sin la placa).

    python tests_host/run_tests.py

Necesita un compilador de C++. Usa, en este orden: g++, clang++ o el clang empaquetado
de PyPI (python -m pip install --user ziglang). Ver tests_host/README.md.
"""
from __future__ import annotations

import pathlib
import shutil
import subprocess
import sys

RAIZ = pathlib.Path(__file__).resolve().parents[1]
SRC = RAIZ / "src"
BUILD = RAIZ / "tests_host" / "build"
FUENTES = [
    RAIZ / "tests_host" / "test_senoide.cpp",
    SRC / "motion_profile" / "sine_profile.cpp",
    SRC / "motion_profile" / "trapezoidal_profile.cpp",
    SRC / "command_parser" / "command_parser.cpp",
    SRC / "sine_command.cpp",
]


def compilador():
    for c in ("g++", "clang++"):
        if shutil.which(c):
            return [c]
    try:
        subprocess.run([sys.executable, "-m", "ziglang", "version"], check=True, capture_output=True)
        return [sys.executable, "-m", "ziglang", "c++"]
    except (subprocess.CalledProcessError, FileNotFoundError):
        sys.exit("No hay compilador de C++: instalar g++/clang o 'python -m pip install --user ziglang'")


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    exe = BUILD / ("test_senoide.exe" if sys.platform == "win32" else "test_senoide")
    print("== compilando tests de C++", flush=True)
    subprocess.run(compilador() + ["-std=c++17", "-O2", "-Wall", f"-I{SRC}", *map(str, FUENTES), "-o", str(exe)],
                   check=True)
    csv = BUILD / "barrido_m60.csv"
    print("== tests de integración del M60 (C++)", flush=True)
    r1 = subprocess.run([str(exe), "--csv", str(csv)]).returncode
    print("== punta a punta: consigna del firmware -> máquina simulada -> análisis de la prueba 4", flush=True)
    r2 = subprocess.run([sys.executable, str(RAIZ / "tests_host" / "test_e2e_prueba4.py"), str(csv)]).returncode
    ok = r1 == 0 and r2 == 0
    print("== RESULTADO:", "PASAN" if ok else "FALLAN")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
