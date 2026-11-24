"""Búsqueda del puerto serie de la placa (Linux, macOS y Windows)."""
import glob
import os

import serial.tools.list_ports

VID_RASPBERRY_PI = 0x2E8A     # Raspberry Pi (Pico con el core de Arduino)


def candidatos():
    """Puertos serie posibles, la Pico primero.

    En Linux se prefiere /dev/serial/by-id/..., que no cambia si la placa re-enumera.
    """
    pico, otros = [], []
    pico += sorted(glob.glob("/dev/serial/by-id/*Pico*"))
    for p in serial.tools.list_ports.comports():
        if p.vid == VID_RASPBERRY_PI or "pico" in (p.description or "").lower():
            pico.append(p.device)
        elif any(s in p.device for s in ("ACM", "USB", "usbmodem", "COM")):
            otros.append(p.device)
    otros += sorted(glob.glob("/dev/serial/by-id/*"))
    return _sin_repetir(pico + otros)


def picos():
    """Solo los puertos que son una Raspberry Pi Pico."""
    out = sorted(glob.glob("/dev/serial/by-id/*Pico*"))
    out += [p.device for p in serial.tools.list_ports.comports() if p.vid == VID_RASPBERRY_PI]
    return _sin_repetir(out)


def _sin_repetir(ports):
    """El enlace /dev/serial/by-id/... y /dev/ttyACMx son la misma placa: queda el primero."""
    vistos, salida = set(), []
    for p in ports:
        real = os.path.realpath(p)
        if real not in vistos:
            vistos.add(real)
            salida.append(p)
    return salida


def elegir(port=None):
    """El puerto pedido, o la primera Pico que aparezca. ``None`` si no hay ninguna."""
    if port:
        return port
    p = picos()
    return p[0] if p else None
