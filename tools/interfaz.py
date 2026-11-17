#!/usr/bin/env python3
"""Interfaz gráfica del eje: control remoto y telemetría en vivo.

    python3 tools/interfaz.py                    # busca la placa y se conecta sola
    python3 tools/interfaz.py --port /dev/ttyACM1

Guía de uso: docs/GUIA_INTERFAZ.md. Cerrar antes cualquier otro programa que tenga el
puerto abierto (monitor serie de PlatformIO, otros scripts de tools/).

No cambia nada del firmware: manda los mismos comandos G-code que se escriben a mano y
prende la telemetría binaria (M120). La pestaña «Encoder» corre el barrido de caracterización
del encoder con M57 … R1 (firmware del 17/11/2026 en adelante) y lo grafica como
tools/encoder_sweep.py y tools/plot_periods.py. Protocolo (ver src/command_processor.cpp y
src/shared_data.h): una respuesta por comando, terminada en "ok" o "error: ...", con los
paquetes de telemetría mezclados en el mismo puerto: 0xAA 0x56 + 28 bytes little endian
    uint32 t_us, int32 target [0,25 nm], int32 pos [0,25 nm], float vel [rad/s], float out [rad],
    int32 chip_raw, uint8 status, uint8 flags, uint16 seq
(hasta el 23/10/2026 era 0xAA 0x55 con objetivo y posición en float [mm]: de a 15 nm a 200 mm)
"""
import argparse
import datetime
import glob
import math
import os
import queue
import struct
import sys
import threading
import time

import numpy as np
import serial
import serial.tools.list_ports


def _clean_snap_env():
    """La terminal de VS Code instalado como snap exporta rutas de GTK/GLib del snap, y con esas
    Qt se cae al arrancar ("Settings schema ... does not contain a key named 'antialiasing'").
    Se vuelven a las del sistema antes de cargar Qt."""
    if "/snap/" not in os.environ.get("GSETTINGS_SCHEMA_DIR", "") + os.environ.get("GTK_PATH", ""):
        return
    for var in ("XDG_DATA_DIRS", "XDG_CONFIG_DIRS"):
        orig = os.environ.get(var + "_VSCODE_SNAP_ORIG")
        if orig is not None:
            os.environ[var] = orig
    for var in ("GSETTINGS_SCHEMA_DIR", "GTK_PATH", "GTK_EXE_PREFIX", "GIO_MODULE_DIR", "GTK_IM_MODULE_FILE",
                "GDK_PIXBUF_MODULE_FILE", "GDK_PIXBUF_MODULEDIR", "LOCPATH"):
        os.environ.pop(var, None)


_clean_snap_env()
from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402
from PyQt5.QtCore import Qt  # noqa: E402
import pyqtgraph as pg  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import encoder_analysis as ea  # noqa: E402

# --- montaje y firmware (ver src/hw_config.h) ---
BAUD = 921600
PITCH_MM = 5.0                 # husillo: mm por vuelta de motor
AXIS_MIN_MM, AXIS_MAX_MM = 0.0, 255.0
POS_DECIMALS = 6              # 1 nm en destino, G1 e indicadores (el firmware lee el G1 en double)
POS_PLOT_MIN_SPAN_MM = 5e-5    # 50 nm: con el eje quieto el autoescalado no hace zoom al ruido de
                              # nm (la telemetría viene de a 0,25 nm) pero se ve un paso de 10 nm
SPEED_DEFAULT = 2.0            # mm/s: la velocidad para la que se aprende el anti-cogging
SPEED_SAFE = 2.0               # por encima el anti-cogging se desvanece (no está a 3 mm/s)
SPEED_MAX = 5.0                # MAX_VELOCITY_LIMIT_MM_S (a 4 mm/s ya falta torque)
OUT_LIMIT_DEG = 81.0           # límite de la salida del lazo de velocidad (0.45 pi)
STEP_EXP_MIN = -POS_DECIMALS   # paso más chico: la resolución del G1
STEP_EXP_MAX = 1               # 10 mm
TELEMETRY_DECIMATION = 2       # M120 P2: un punto cada 2 ticks de la ISR de 2 kHz = 1 kHz
HISTORY_S = 130                # lo que se guarda para los gráficos
STOP_LEAD_S = 0.015            # Detener: frena donde va a estar el objetivo cuando llegue el G1

PKT = struct.Struct("<IiiffiBBH")
HEADER = b"\xaa\x56"
POS_LSB_MM = 0.25e-6           # unidad del objetivo y la posición en el paquete (POSITION_LSB_MM)
STATUS_BITS = {0x01: "sobrevelocidad", 0x02: "campo débil", 0x04: "subtensión", 0x08: "error CRC"}
LOG_PREFIXES = ("I)", "W)", "E)", "D)")
# comandos que bloquean el firmware un buen rato (el resto responde en milisegundos)
LONG_TIMEOUTS = {"M56": 900.0, "G28": 240.0, "M57": 300.0, "M121": 30.0}
LONG_LABELS = {"M56": "Calibrando (M56)", "G28": "Homing (G28)", "M57": "Barrido en lazo abierto (M57)"}

# --- barrido del encoder (pestaña Encoder, ver tools/encoder_analysis.py) ---
MOTOR_POLE_PAIRS = 50          # si el META del barrido no lo trae
WHEEL_MAGNETS = 60             # rueda de imanes: 60 imanes = 30 períodos del encoder por vuelta
WHEEL_PITCH_MM = 3.0           # paso entre imanes de la rueda
SWEEP_REVS_DEFAULT = 2         # vueltas de motor por sentido, como el firmware de barrido (G56)
SWEEP_SPEED_DEFAULT = 1.25     # mm/s de carro = 0,25 vueltas/s, también como G56
SWEEP_DECIMATION = 2           # M57 lee el encoder cada ~0,5 ms: una muestra por ms

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# --- estilo (mismos colores que los gráficos de tools/closed_loop_test.py) ---
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
BLUE = "#2a78d6"
ORANGE = "#eb6834"
RED = "#d03b3b"
GREEN = "#2e9e5b"
AMBER = "#d99a1e"
PASS_COLORS = (BLUE, GREEN, "#7a5cc7", INK2)   # pasada 1, 2, ... en las ventanas de 2 períodos
# referencias de color en los títulos de la pestaña Encoder (los puntos de 2 px no se ven en una leyenda)
KEY_DIRS = f"<span style='color:{BLUE}'>■</span> ida &nbsp;<span style='color:{ORANGE}'>■</span> vuelta"
KEY_WEAK = f" &nbsp;<span style='color:{RED}'>✕</span> campo débil"
KEY_JUMP = f" &nbsp;<span style='color:{RED}'>✕</span> salto (&gt; 45° entre muestras)"


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def find_ports():
    """Puertos serie candidatos, la Pico primero (por id, que no cambia si re-enumera)."""
    ports = sorted(glob.glob("/dev/serial/by-id/*Pico*")) + sorted(glob.glob("/dev/serial/by-id/*"))
    for p in serial.tools.list_ports.comports():
        if "ACM" in p.device or "USB" in p.device or p.device.startswith("COM"):
            ports.append(p.device)
    seen, out = set(), []
    for p in ports:
        if p not in seen:
            seen.add(p)
            out.append(p)
    return out


def parse_status(lines):
    """M119 -> dict con homed, calibrated, target_mm, pos_mm, fault, motor_update..."""
    st = {}
    for line in lines:
        if line.startswith("ultimo corte"):
            continue
        for tok in line.replace("servo:", "").split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                st[k] = v
    return st


# ======================================================================================
# Enlace serie
# ======================================================================================

class Command:
    def __init__(self, text, silent=False, on_done=None):
        self.text = text.strip()
        self.code = self.text.split()[0].upper() if self.text else ""
        self.silent = silent           # no mostrar en la consola (sondeos de estado)
        self.on_done = on_done
        self.timeout = LONG_TIMEOUTS.get(self.code, 5.0)
        self.lines = []
        self.ok = None                 # True: ok, False: error, None: sin respuesta / descartado


class Link(QtCore.QObject):
    """Puerto serie con un hilo lector (texto + telemetría) y uno que manda los comandos de a
    uno, esperando el ok/error de cada uno antes de mandar el siguiente."""

    console = QtCore.pyqtSignal(str, str)      # texto, tipo: cmd | reply | log
    done = QtCore.pyqtSignal(object)            # Command terminado
    lost = QtCore.pyqtSignal(str)               # se perdió el puerto

    def __init__(self):
        super().__init__()
        self.ser = None
        self.port = None
        self._running = False
        self._rows = []
        self._rows_lock = threading.Lock()
        self._lines = queue.Queue()
        self._cmds = queue.Queue()
        self._buf = bytearray()
        self._text = ""
        self.current = None            # comando esperando respuesta

    @property
    def is_open(self):
        return self.ser is not None and self._running

    def open(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.02, write_timeout=1.0, exclusive=True)
        self.port = port
        self.ser.reset_input_buffer()
        self._buf.clear()
        self._text = ""
        self._running = True
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._writer = threading.Thread(target=self._cmd_loop, daemon=True)
        self._reader.start()
        self._writer.start()

    def close(self, stop_telemetry=True):
        if self.ser is None:
            return
        self._running = False
        for th in (self._writer, self._reader):
            th.join(timeout=1.0)
        try:
            if stop_telemetry:
                self.ser.write(b"M120 S0\n")
                self.ser.flush()
                time.sleep(0.1)
            self.ser.close()
        except (serial.SerialException, OSError):
            pass
        self.ser = None
        self.current = None
        self._drop_pending()

    # --- comandos ---
    def send(self, text, silent=False, on_done=None):
        cmd = Command(text, silent, on_done)
        self._cmds.put(cmd)
        return cmd

    def send_urgent(self, texts, silent=False):
        """Descarta lo que está en cola y manda estos comandos primero."""
        self._drop_pending()
        for t in texts:
            self.send(t, silent)

    def pending(self):
        return self._cmds.qsize() + (1 if self.current is not None else 0)

    def _drop_pending(self):
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                return
            cmd.lines.append("descartado")
            self.done.emit(cmd)

    def take_rows(self):
        with self._rows_lock:
            rows, self._rows = self._rows, []
        return rows

    # --- hilos ---
    def _read_loop(self):
        try:
            while self._running:
                n = self.ser.in_waiting
                data = self.ser.read(n if n else 1)
                if data:
                    self._parse(data)
        except (serial.SerialException, OSError, TypeError) as e:
            if self._running:
                self._running = False
                self.lost.emit(str(e))

    def _parse(self, data):
        buf = self._buf
        buf += data
        rows = []
        while True:
            i = buf.find(HEADER)
            if i < 0:
                # el texto es ASCII: 0xAA solo puede ser el comienzo de un paquete
                keep = 1 if buf.endswith(b"\xaa") else 0
                self._add_text(buf[:len(buf) - keep])
                del buf[:len(buf) - keep]
                break
            if i > 0:
                self._add_text(buf[:i])
                del buf[:i]
            if len(buf) < 2 + PKT.size:
                break
            t_us, target_q, pos_q, *rest = PKT.unpack_from(buf, 2)
            row = (t_us, target_q * POS_LSB_MM, pos_q * POS_LSB_MM, *rest)
            if all(math.isfinite(v) and abs(v) < 1e6 for v in row[1:5]):
                rows.append(row)
                del buf[:2 + PKT.size]
            else:
                del buf[:1]   # no era un paquete: resincronizar
        if rows:
            with self._rows_lock:
                self._rows.extend(rows)

    def _add_text(self, b):
        if not b:
            return
        self._text += bytes(b).decode(errors="replace")
        while "\n" in self._text:
            line, self._text = self._text.split("\n", 1)
            line = line.strip("\r\x00 ")
            if line:
                self._lines.put(line)

    def _flush_async(self):
        while True:
            try:
                line = self._lines.get_nowait()
            except queue.Empty:
                return
            self.console.emit(line, "log")

    def _cmd_loop(self):
        while self._running:
            try:
                cmd = self._cmds.get(timeout=0.05)
            except queue.Empty:
                self._flush_async()
                continue
            self._flush_async()
            self.current = cmd
            if not cmd.silent:
                self.console.emit(cmd.text, "cmd")
            try:
                self.ser.write((cmd.text + "\n").encode())
                deadline = time.monotonic() + cmd.timeout
                while self._running and time.monotonic() < deadline:
                    try:
                        line = self._lines.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if line.startswith(LOG_PREFIXES):
                        self.console.emit(line, "log")
                        continue
                    cmd.lines.append(line)
                    if not cmd.silent:
                        self.console.emit(line, "reply")
                    if line.startswith("ok"):
                        cmd.ok = True
                        break
                    if line.startswith("error"):
                        cmd.ok = False
                        break
                if cmd.ok is None and self._running:
                    cmd.lines.append(f"sin respuesta en {cmd.timeout:g} s")
            except (serial.SerialException, OSError) as e:
                cmd.lines.append(f"error de escritura: {e}")
            self.current = None
            self.done.emit(cmd)


# ======================================================================================
# Historia de la telemetría
# ======================================================================================

class History:
    """Últimos HISTORY_S segundos de telemetría en arreglos contiguos (buffer doble)."""
    T, TARGET, POS, VEL, OUT, STATUS, FLAGS = range(7)

    def __init__(self, cap):
        self.cap = cap
        self.data = np.zeros((7, 2 * cap))
        self.n = 0
        self.t = 0.0                   # tiempo de la placa, desenrollado [s]
        self.t_last_us = None
        self.seq_last = None
        self.lost = 0                  # puntos perdidos en el camino (huecos de seq)
        self.received = 0

    def clear(self):
        self.n = 0

    def add(self, rows):
        a = np.array(rows, dtype=np.float64)
        t_us = a[:, 0].astype(np.int64)
        prev = self.t_last_us if self.t_last_us is not None else t_us[0]
        dt = np.diff(np.concatenate(([prev], t_us))) % (1 << 32)
        t = self.t + np.cumsum(dt) / 1e6
        seq = a[:, 8].astype(np.int64)
        prev_seq = self.seq_last if self.seq_last is not None else seq[0] - 1
        dseq = np.diff(np.concatenate(([prev_seq], seq))) % (1 << 16)
        # los huecos largos (firmware ocupado en M56/G28, que no vacía la cola) no cuentan
        self.lost += int(np.sum(np.where(dt < 200000, np.maximum(dseq - 1, 0), 0)))
        self.t, self.t_last_us, self.seq_last = t[-1], t_us[-1], seq[-1]
        self.received += len(a)

        new = np.empty((7, len(a)))
        new[self.T] = t
        new[self.TARGET] = a[:, 1]
        new[self.POS] = a[:, 2]
        new[self.VEL] = a[:, 3] / (2 * np.pi) * PITCH_MM                # rad/s -> mm/s
        new[self.OUT] = np.degrees(a[:, 4])
        new[self.STATUS] = a[:, 6]
        new[self.FLAGS] = a[:, 7]
        new = new[:, -self.cap:]
        k = new.shape[1]
        if self.n + k > 2 * self.cap:
            keep = min(self.n, self.cap - k)
            self.data[:, :keep] = self.data[:, self.n - keep:self.n]
            self.n = keep
        self.data[:, self.n:self.n + k] = new
        self.n += k
        return new

    def last(self, seconds):
        """Vista de los datos de los últimos 'seconds' segundos."""
        t = self.data[self.T, :self.n]
        i = np.searchsorted(t, self.t - seconds) if self.n else 0
        return self.data[:, i:self.n]


# ======================================================================================
# Widgets
# ======================================================================================

def fmt_len(exp):
    """10^exp mm en la unidad que le queda: 100 nm, 10 µm, 1 mm."""
    unit, e = ("mm", exp) if exp >= 0 else ("µm", exp + 3) if exp >= -3 else ("nm", exp + 6)
    return f"{10 ** e} {unit}"


class SpinBox(QtWidgets.QDoubleSpinBox):
    """Acepta coma o punto como separador decimal."""

    def __init__(self, lo, hi, decimals, step, suffix):
        super().__init__()
        self.setLocale(QtCore.QLocale.c())
        self.setRange(lo, hi)
        self.setDecimals(decimals)
        self.setSingleStep(step)
        self.setSuffix(suffix)
        self.setKeyboardTracking(False)
        self.setAccelerated(True)

    def validate(self, text, pos):
        return super().validate(text.replace(",", "."), pos)

    def valueFromText(self, text):
        return super().valueFromText(text.replace(",", "."))


class OffsetAxis(pg.AxisItem):
    """Eje de posición para datos graficados respecto de `offset` (mm, un número redondo).

    Con zoom de µm o nm sobre ~150 mm los rótulos absolutos (149.99996) no entran en el eje y
    pyqtgraph los esconde: con menos de 10 µm entre marcas rotula en µm sobre `offset`.
    `on_reference(texto)` avisa cuándo mostrar la referencia (texto vacío: rótulos absolutos).
    """

    def __init__(self, unit, on_reference):
        super().__init__("left")
        self.unit = unit
        self.on_reference = on_reference
        self.offset = 0.0
        self.offset_places = 0
        self.relative = False

    def set_offset(self, offset, step):
        if offset != self.offset:
            self.offset = offset
            self.offset_places = max(0, -round(math.log10(step)))
            self._update_label()
            self.picture = None
            self.update()

    def _update_label(self):
        text = "µm" if self.relative else self.unit
        if self.labelText != text:
            self.setLabel(text)
        self.on_reference(f"µm sobre {self.offset:.{self.offset_places}f} {self.unit}" if self.relative else "")

    def tickValues(self, minVal, maxVal, size):
        # se decide acá y no en tickStrings, que se llama por separado para mayores y menores
        levels = super().tickValues(minVal, maxVal, size)
        relative = bool(levels) and min(spacing for spacing, _ in levels) < 1e-2
        if relative != self.relative:
            self.relative = relative
            self._update_label()
        return levels

    def tickStrings(self, values, scale, spacing):
        if self.relative:
            places = max(0, math.ceil(-math.log10(spacing * 1000)))
            return [f"{v * 1000:.{places}f}" for v in values]
        places = max(0, math.ceil(-math.log10(spacing)))
        return [f"{self.offset + v:.{places}f}" for v in values]


class RailWidget(QtWidgets.QWidget):
    """El carril de 0 a 255 mm con el carro (azul) y el destino (triángulo naranja).
    Un clic elige ese punto como destino (no mueve nada)."""

    picked = QtCore.pyqtSignal(float)
    MARGIN = 26

    def __init__(self):
        super().__init__()
        self.pos_mm = None
        self.dest_mm = None
        self.setMinimumHeight(86)
        self.setCursor(Qt.PointingHandCursor)
        self.setToolTip("Clic en el carril: elegir ese punto como destino (después «Ir»)")

    def set_state(self, pos_mm, dest_mm):
        self.pos_mm, self.dest_mm = pos_mm, dest_mm
        self.update()

    def _x(self, mm):
        return self.MARGIN + (self.width() - 2 * self.MARGIN) * (mm - AXIS_MIN_MM) / (AXIS_MAX_MM - AXIS_MIN_MM)

    def mousePressEvent(self, ev):
        frac = (ev.x() - self.MARGIN) / max(1, self.width() - 2 * self.MARGIN)
        mm = clamp(AXIS_MIN_MM + frac * (AXIS_MAX_MM - AXIS_MIN_MM), AXIS_MIN_MM, AXIS_MAX_MM)
        self.picked.emit(round(mm, 1))

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        h = self.height()
        y = int(h * 0.42)
        x0, x1 = self._x(AXIS_MIN_MM), self._x(AXIS_MAX_MM)
        # topes y riel
        p.setPen(Qt.NoPen)
        p.setBrush(QtGui.QColor(GRID))
        p.drawRoundedRect(QtCore.QRectF(x0, y - 4, x1 - x0, 8), 4, 4)
        p.setBrush(QtGui.QColor(MUTED))
        p.drawRect(QtCore.QRectF(x0 - 6, y - 14, 4, 28))
        p.drawRect(QtCore.QRectF(x1 + 2, y - 14, 4, 28))
        # marcas
        font = p.font()
        font.setPointSizeF(8)
        p.setFont(font)
        for mm in range(0, 256, 10):
            x = self._x(mm)
            big = mm % 50 == 0
            p.setPen(QtGui.QPen(QtGui.QColor(MUTED if big else GRID), 1))
            p.drawLine(QtCore.QPointF(x, y + 8), QtCore.QPointF(x, y + (16 if big else 12)))
            if big and mm < 250:
                p.setPen(QtGui.QColor(INK2))
                p.drawText(QtCore.QRectF(x - 20, y + 17, 40, 14), Qt.AlignCenter, f"{mm}")
        p.setPen(QtGui.QColor(INK2))
        p.drawText(QtCore.QRectF(x1 - 40, y + 17, 64, 14), Qt.AlignRight, "255 mm")
        # destino
        if self.dest_mm is not None:
            x = self._x(self.dest_mm)
            p.setPen(Qt.NoPen)
            p.setBrush(QtGui.QColor(ORANGE))
            p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(x, y - 18), QtCore.QPointF(x - 7, y - 30),
                                           QtCore.QPointF(x + 7, y - 30)]))
        # carro
        if self.pos_mm is not None:
            x = self._x(clamp(self.pos_mm, AXIS_MIN_MM - 3, AXIS_MAX_MM + 3))
            p.setPen(QtGui.QPen(QtGui.QColor("#184f95"), 1))
            p.setBrush(QtGui.QColor(BLUE))
            p.drawRoundedRect(QtCore.QRectF(x - 9, y - 13, 18, 26), 3, 3)
            p.setPen(QtGui.QPen(QtGui.QColor("white"), 1.5))
            p.drawLine(QtCore.QPointF(x, y - 8), QtCore.QPointF(x, y + 8))
        p.end()


class Readout(QtWidgets.QFrame):
    def __init__(self, title, unit, tooltip=""):
        super().__init__()
        self.setObjectName("readout")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)
        lay.setSpacing(0)
        top = QtWidgets.QLabel(f"{title} <span style='color:{MUTED}'>[{unit}]</span>")
        top.setObjectName("readoutTitle")
        self.value = QtWidgets.QLabel("—")
        self.value.setObjectName("readoutValue")
        self.value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(top)
        lay.addWidget(self.value)
        self.setToolTip(tooltip)

    def set(self, text, color=INK):
        self.value.setText(text)
        if color != getattr(self, "_color", None):
            self._color = color
            self.value.setStyleSheet(f"color:{color}")


class Chip(QtWidgets.QLabel):
    def __init__(self, text, tooltip=""):
        super().__init__(text)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(92)
        self.setToolTip(tooltip)
        self.set_state(None)

    def set_state(self, state):
        """True: verde, False: rojo, None: gris (desconocido), 'warn': ámbar."""
        if getattr(self, "_state", object()) == state:
            return
        self._state = state
        bg, fg = {True: (GREEN, "white"), False: (RED, "white"), "warn": (AMBER, "white")}.get(
            state, ("#e9e8e3", MUTED))
        self.setStyleSheet(f"background:{bg}; color:{fg}; border-radius:10px; padding:3px 10px; font-weight:600")


class GcodeLine(QtWidgets.QLineEdit):
    """Entrada de G-code con historia (flechas arriba y abajo)."""

    def __init__(self):
        super().__init__()
        self.history = []
        self.idx = 0
        self.setPlaceholderText("G-code, p. ej. M119   (Enter para mandar, ↑↓ historia)")

    def push(self, text):
        if text and (not self.history or self.history[-1] != text):
            self.history.append(text)
        self.idx = len(self.history)

    def keyPressEvent(self, ev):
        if ev.key() == Qt.Key_Up and self.history:
            self.idx = max(0, self.idx - 1)
            self.setText(self.history[self.idx])
        elif ev.key() == Qt.Key_Down and self.history:
            self.idx = min(len(self.history), self.idx + 1)
            self.setText(self.history[self.idx] if self.idx < len(self.history) else "")
        else:
            super().keyPressEvent(ev)


class KeyFilter(QtCore.QObject):
    """Atajos: ←/→ un paso, ↑/↓ paso ×10 / ÷10, Espacio detener, Esc apagar el motor. Las flechas y el espacio no
    se toman mientras se escribe en un campo de texto o número. En la pestaña Encoder, ←/→ cambian de ventana
    (Esc sigue apagando el motor)."""

    def __init__(self, win):
        super().__init__()
        self.win = win

    def eventFilter(self, obj, ev):
        if ev.type() != QtCore.QEvent.KeyPress or not isinstance(obj, QtWidgets.QWidget):
            return False
        if obj.window() is not self.win:
            return False
        key = ev.key()
        if key == Qt.Key_Escape:
            self.win.motor_off()
            return True
        if isinstance(QtWidgets.QApplication.focusWidget(), (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox)):
            return False
        if self.win.tabs.currentWidget() is self.win.enc_tab:
            # en la pestaña Encoder las flechas recorren las ventanas, no mueven el carro
            if key in (Qt.Key_Left, Qt.Key_Right):
                self.win.enc_tab.step_window(+1 if key == Qt.Key_Right else -1)
                return True
            return False
        if key in (Qt.Key_Left, Qt.Key_Right, Qt.Key_Space):
            if not ev.isAutoRepeat():
                if key == Qt.Key_Space:
                    self.win.stop_motion()
                else:
                    self.win.jog(+1 if key == Qt.Key_Right else -1)
            return True
        if key in (Qt.Key_Up, Qt.Key_Down):
            self.win.step_scale(+1 if key == Qt.Key_Up else -1)
            return True
        return False


# ======================================================================================
# Pestaña «Encoder»: barrido de caracterización (encoder_sweep.py + plot_periods.py)
# ======================================================================================

def _sweep_csvs():
    """CSV de barridos guardados en mediciones/, el más nuevo primero."""
    files = glob.glob(os.path.join(REPO, "mediciones", "*", "encoder_sweep_*.csv"))
    return sorted(files, key=lambda p: os.path.basename(p), reverse=True)


def _sweep_label(path):
    name = os.path.splitext(os.path.basename(path))[0]
    try:
        ts = datetime.datetime.strptime(name.replace("encoder_sweep_", ""), "%Y%m%d_%H%M%S")
        return f"{ts:%d/%m/%Y  %H:%M:%S}"
    except ValueError:
        return name


def _pass_index(T):
    """Número de pasada (0, 1, ...) dentro de cada sentido, contado desde donde arranca ese sentido."""
    p = np.zeros(len(T["dir"]), int)
    for s in (+1, -1):
        m = T["dir"] == s
        if m.any():
            md = T["motor_deg"][m]
            n = max(1, int(round(np.ptp(md) / 360.0)))
            p[m] = np.minimum(np.floor(np.abs(md - md[0]) / 360.0 + 1e-9).astype(int), n - 1)
    return p


class EncoderTab(QtWidgets.QWidget):
    """Corre el barrido en lazo abierto con el firmware real (M57 R1: ida y vuelta, N vueltas de
    motor) y muestra lo mismo que tools/encoder_sweep.py y tools/plot_periods.py, con zoom."""

    BUSY_LABEL = "Barrido del encoder (M57 R1)"

    def __init__(self, win):
        super().__init__()
        self.win = win
        self.proc = None               # QProcess de «Exportar PNG»
        self.table = None              # columnas de ea.COLS
        self.T = None
        self.stats = None
        self.windows = None
        self.cur_win = 0
        self.from_home = False         # motor_deg cuenta desde el 0 del eje (M57) o desde el inicio (G56)
        self.path = None
        # barrido en curso
        self.running = False
        self.legs = []
        self.leg_i = 0
        self._cmd = None
        self._seen = 0
        self.rows = []
        self.meta = {}
        self.leg_field0 = None
        self._live_dirty = False
        self._pending_select = None

        self._build()
        self.refresh_list()
        self.live_timer = QtCore.QTimer(self, timeout=self._tick_live)
        self.live_timer.start(250)

    # --- UI ---------------------------------------------------------------------------
    def _build(self):
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 4, 0, 0)
        lay.setSpacing(8)

        left = QtWidgets.QWidget()
        ll = QtWidgets.QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 4, 0)
        ll.setSpacing(8)

        box, b = self.win._group("Barrido del encoder")
        intro = QtWidgets.QLabel(
            "Gira el motor en <b>lazo abierto</b>, ida y vuelta, y registra el ángulo crudo del MT6835 "
            "contra el campo comandado (<code>M57 … R1</code>): ¿la rueda y el sensor están bien puestos?")
        intro.setWordWrap(True)
        intro.setStyleSheet(f"color:{INK2}; font-weight:400")
        b.addWidget(intro)
        form = QtWidgets.QGridLayout()
        form.setHorizontalSpacing(8)
        self.spin_revs = QtWidgets.QSpinBox()
        self.spin_revs.setRange(1, 4)
        self.spin_revs.setValue(SWEEP_REVS_DEFAULT)
        self.spin_revs.setSuffix(" vueltas")
        self.spin_revs.setToolTip(f"Vueltas de motor en cada sentido (1 vuelta = {PITCH_MM:g} mm de carro)")
        self.spin_sweep_speed = SpinBox(0.25, 2.0, 2, 0.25, " mm/s")
        self.spin_sweep_speed.setValue(SWEEP_SPEED_DEFAULT)
        self.spin_sweep_speed.setToolTip("Velocidad del campo. 1,25 mm/s = 0,25 vueltas/s, la del firmware de\n"
                                         "barrido (G56): así se compara con las mediciones de octubre.\n"
                                         "M57 no acepta más de 2 mm/s.")
        form.addWidget(QtWidgets.QLabel("Recorrido"), 0, 0)
        form.addWidget(self.spin_revs, 0, 1)
        form.addWidget(QtWidgets.QLabel("Velocidad"), 1, 0)
        form.addWidget(self.spin_sweep_speed, 1, 1)
        form.setColumnStretch(1, 1)
        b.addLayout(form)
        self.btn_run = QtWidgets.QPushButton("▶  Correr barrido")
        self.btn_run.setObjectName("primary")
        self.btn_run.clicked.connect(self.start)
        b.addWidget(self.btn_run)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(10)
        self.lbl_run = QtWidgets.QLabel()
        self.lbl_run.setWordWrap(True)
        self.lbl_run.setStyleSheet(f"color:{MUTED}; font-weight:400")
        b.addWidget(self.progress)
        b.addWidget(self.lbl_run)
        ll.addWidget(box)

        box, b = self.win._group("Mediciones guardadas")
        self.list = QtWidgets.QListWidget()
        self.list.setMaximumHeight(110)
        self.list.setToolTip("mediciones/AAAA-MM-DD/encoder_sweep_*.csv — clic para verla")
        self.list.currentItemChanged.connect(self._list_picked)
        b.addWidget(self.list)
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(4)
        btn_open = QtWidgets.QPushButton("Abrir CSV…", objectName="seg")
        btn_open.clicked.connect(self.open_dialog)
        btn_ref = QtWidgets.QPushButton("↻", objectName="seg", toolTip="Volver a buscar en mediciones/")
        btn_ref.clicked.connect(lambda: self.refresh_list())
        self.btn_png = QtWidgets.QPushButton("Exportar PNG", objectName="seg")
        self.btn_png.setToolTip("Corre tools/encoder_sweep.py --from-csv y tools/plot_periods.py sobre esta\n"
                                "medición: deja los PNG de siempre junto al CSV (para el informe).")
        self.btn_png.clicked.connect(self.export_png)
        row.addWidget(btn_open)
        row.addWidget(btn_ref)
        row.addStretch(1)
        row.addWidget(self.btn_png)
        b.addLayout(row)
        ll.addWidget(box)

        box, b = self.win._group("Resultado")
        self.lbl_src = QtWidgets.QLabel("Correr un barrido o elegir una medición guardada.")
        self.lbl_src.setWordWrap(True)
        self.lbl_src.setStyleSheet(f"color:{MUTED}; font-weight:400")
        b.addWidget(self.lbl_src)
        self.lbl_verdict = QtWidgets.QLabel()
        self.lbl_verdict.setWordWrap(True)
        self.lbl_verdict.hide()
        b.addWidget(self.lbl_verdict)
        grid = QtWidgets.QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(5)
        self.res = {}
        for i, (key, title, tip) in enumerate((
                ("ppr", "Períodos por vuelta", "Pendiente del ángulo desenrollado contra la posición comandada (ida).\n"
                                               "Con 60 imanes tiene que dar 30,00: si no, la lectura se\n"
                                               "corrió (se perdieron períodos) o el motor patinó."),
                ("jumps", "Saltos", "Muestras seguidas cuyo paso difiere en más de 45° del esperado"),
                ("weak", "Campo débil", "Muestras con el bit de campo débil del MT6835"),
                ("ida", "Error ida", "Ángulo medido menos la recta ideal, en grados de encoder (1 período =\n"
                                     "360°). Incluye el atraso del rotor en lazo abierto. Entre paréntesis,\n"
                                     "en grados de eje (÷ 30)."),
                ("vuelta", "Error vuelta", "Igual que la ida, en el sentido contrario"),
                ("bits", "Sobrevel. / CRC / subt.", "Bits de estado del chip: sobrevelocidad, error de CRC, subtensión"),
                ("n", "Muestras", "Cantidad, duración y vueltas recorridas"))):
            t = QtWidgets.QLabel(title)
            t.setStyleSheet(f"color:{INK2}; font-weight:400")
            t.setToolTip(tip)
            v = QtWidgets.QLabel("—")
            v.setObjectName("resValue")
            v.setToolTip(tip)
            v.setTextInteractionFlags(Qt.TextSelectableByMouse)
            v.setWordWrap(True)
            grid.addWidget(t, i, 0)
            grid.addWidget(v, i, 1)
            self.res[key] = v
        grid.setColumnStretch(1, 1)
        b.addLayout(grid)
        ll.addWidget(box)
        ll.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(left)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFixedWidth(410)
        scroll.setStyleSheet(f"QScrollArea, QScrollArea > QWidget > QWidget {{ background: {SURFACE}; }}")
        lay.addWidget(scroll)

        self.sub = QtWidgets.QTabWidget()
        self.sub.addTab(self._build_sweep_plots(), "A lo largo del barrido")
        self.sub.addTab(self._build_window_plots(), "Ventanas de 2 períodos")
        self.sub.addTab(self._build_phase_plots(), "Por fase e imán")
        self.sub.addTab(self._build_status_plots(), "Saltos y estado")
        lay.addWidget(self.sub, 1)

    def _plot(self, glw, row, col, title, xlabel, ylabel, **kw):
        p = glw.addPlot(row=row, col=col, **kw)
        MainWindow._set_title(p, title)
        p.setLabel("bottom", xlabel)
        p.setLabel("left", ylabel)
        p.showGrid(x=True, y=True, alpha=0.25)
        font = QtGui.QFont()
        font.setPointSize(8)
        for ax in ("left", "bottom"):
            p.getAxis(ax).setTickFont(font)
            p.getAxis(ax).enableAutoSIPrefix(False)
        p.getAxis("left").setWidth(52)
        return p

    @staticmethod
    def _legend(p):
        """Leyenda con fondo, para que se lea encima de los puntos."""
        return p.addLegend(offset=(8, 4), labelTextColor=INK2, brush=pg.mkBrush(255, 255, 255, 225),
                           pen=pg.mkPen(GRID), verSpacing=-4)

    @staticmethod
    def _dots(p, color, size=2, name=None, symbol="o", alpha=255):
        c = QtGui.QColor(color)
        c.setAlpha(alpha)
        return p.plot([], [], pen=None, symbol=symbol, symbolSize=size, symbolPen=None if symbol == "o" else
                      pg.mkPen(color, width=1.2), symbolBrush=c, name=name)

    def _build_sweep_plots(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)
        hint = QtWidgets.QLabel("Rueda para zoom en x, arrastrar para desplazar, doble clic para ver todo. "
                                "El diente de sierra tiene que ser parejo: 30 períodos por vuelta, sin escalones.")
        hint.setStyleSheet(f"color:{MUTED}; font-weight:400")
        hint.setWordWrap(True)
        lay.addWidget(hint)
        glw = pg.GraphicsLayoutWidget()
        glw.ci.setSpacing(6)
        self.p_ang = self._plot(glw, 0, 0, "Ángulo crudo del encoder", "", "° encoder")
        self.p_errx = self._plot(glw, 1, 0, "Error contra la recta ideal",
                                 "posición del motor [vueltas]", "° encoder")
        self.p_errx.setXLink(self.p_ang)
        for p in (self.p_ang, self.p_errx):
            p.setMouseEnabled(x=True, y=False)
        MainWindow._set_title(self.p_errx, "Error contra la recta ideal", KEY_DIRS + KEY_JUMP)
        self.p_ang.setYRange(-185, 185, padding=0)
        self.c_ang = [self._dots(self.p_ang, BLUE, name="ida"), self._dots(self.p_ang, ORANGE, name="vuelta"),
                      self._dots(self.p_ang, RED, 5, "campo débil", "x")]
        self.p_errx.addLine(y=0, pen=pg.mkPen(GRID, width=1))
        self.c_errx = [self._dots(self.p_errx, BLUE, name="ida"), self._dots(self.p_errx, ORANGE, name="vuelta"),
                       self._dots(self.p_errx, RED, 9, "salto (> 45° entre muestras)", "x")]
        self.rev_lines = []
        glw.scene().sigMouseClicked.connect(lambda ev: self._dbl_autorange(ev, (self.p_ang, self.p_errx)))
        lay.addWidget(glw, 1)
        return w

    def _build_window_plots(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(4, 4, 4, 4)
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Ventana:"))
        self.btn_prev = QtWidgets.QPushButton("◀", objectName="seg", toolTip="Ventana anterior (←)")
        self.btn_next = QtWidgets.QPushButton("▶", objectName="seg", toolTip="Ventana siguiente (→)")
        self.win_combo = QtWidgets.QComboBox()
        self.win_combo.setMinimumWidth(300)
        self.win_combo.currentIndexChanged.connect(self.select_window)
        self.btn_prev.clicked.connect(lambda: self.step_window(-1))
        self.btn_next.clicked.connect(lambda: self.step_window(+1))
        for x in (self.btn_prev, self.win_combo, self.btn_next):
            bar.addWidget(x)
        note = QtWidgets.QLabel("← → cambian de ventana · clic en las barras para ir a una")
        note.setStyleSheet(f"color:{MUTED}; font-weight:400")
        bar.addSpacing(10)
        bar.addWidget(note)
        bar.addStretch(1)
        lay.addLayout(bar)

        ov = pg.GraphicsLayoutWidget()
        ov.setFixedHeight(250)
        ov.ci.setSpacing(4)
        self.p_ov_rms = self._plot(ov, 0, 0, "Error RMS por ventana", "", "° enc.")
        self.p_ov_weak = self._plot(ov, 1, 0, "Campo débil por ventana", "imanes", "%")
        self.p_ov_weak.setXLink(self.p_ov_rms)
        self.ov_items = []
        self.ov_regions = []
        for p in (self.p_ov_rms, self.p_ov_weak):
            p.setMouseEnabled(x=False, y=False)
            p.hideButtons()
            reg = pg.LinearRegionItem(movable=False, brush=pg.mkBrush(42, 120, 214, 28),
                                      pen=pg.mkPen(BLUE, width=1))
            reg.setZValue(-10)
            p.addItem(reg)
            self.ov_regions.append(reg)
        ov.scene().sigMouseClicked.connect(self._ov_clicked)
        lay.addWidget(ov)

        det = pg.GraphicsLayoutWidget()
        det.ci.setSpacing(6)
        self.p_win = [self._plot(det, 0, 0, "Ida", "posición en la rueda [imanes]", "° encoder"),
                      self._plot(det, 0, 1, "Vuelta", "posición en la rueda [imanes]", "° encoder")]
        self.p_win[1].setYLink(self.p_win[0])
        for p in self.p_win:
            self._legend(p)
            p.setYRange(-190, 190, padding=0)
        lay.addWidget(det, 1)
        return w

    def _build_phase_plots(self):
        glw = pg.GraphicsLayoutWidget()
        glw.ci.setSpacing(6)
        self.p_phase = self._plot(glw, 0, 0, "Forma del error dentro de un período",
                                  "fase ideal dentro del período [°]", "° encoder")
        self.p_phase_bits = self._plot(glw, 0, 1, "Dónde aparece el campo débil dentro del período",
                                       "fase ideal dentro del período [°]", "% de muestras")
        self.p_mag = self._plot(glw, 1, 0, "Campo débil a lo largo de la vuelta (1 barra = 1 imán)",
                                "imán", "% de muestras", colspan=2)
        self._legend(self.p_phase_bits)
        MainWindow._set_title(self.p_phase, "Forma del error dentro de un período", KEY_DIRS)
        self.c_phase = [self._dots(self.p_phase, BLUE, name="ida"), self._dots(self.p_phase, ORANGE, name="vuelta")]
        self.phase_items = []
        return glw

    def _build_status_plots(self):
        glw = pg.GraphicsLayoutWidget()
        glw.ci.setSpacing(6)
        self.p_jump = self._plot(glw, 0, 0, "Saltos de la lectura entre muestras seguidas",
                                 "", "medido − esperado [°]")
        self.p_bits = self._plot(glw, 1, 0, "Bits de estado del chip", "tiempo [s]", "")
        self.p_bits.setXLink(self.p_jump)
        self._legend(self.p_jump)
        for y in (ea.JUMP_DEG, -ea.JUMP_DEG):
            self.p_jump.addLine(y=y, pen=pg.mkPen(RED, width=1, style=Qt.DashLine))
        self.c_jump = self.p_jump.plot([], [], pen=pg.mkPen(BLUE, width=1), name="paso medido − esperado")
        self.c_jump_x = self._dots(self.p_jump, RED, 8, "salto", "x")
        names = ["sobrevelocidad", "campo débil", "subtensión", "error CRC"]
        self.p_bits.getAxis("left").setTicks([list(enumerate(names))])
        self.p_bits.getAxis("left").setWidth(100)
        self.p_bits.setYRange(-0.6, 3.6, padding=0)
        self.c_bits = [self._dots(self.p_bits, c, 9, None, "|") for c in (AMBER, RED, INK2, BLUE)]
        return glw

    @staticmethod
    def _dbl_autorange(ev, plots):
        if ev.double():
            for p in plots:
                if p.sceneBoundingRect().contains(ev.scenePos()):
                    p.getViewBox().enableAutoRange(x=True)

    # --- estado de los botones (lo llama MainWindow.update_controls) -------------------
    def update_controls(self):
        w = self.win
        ok = (w.link.is_open and w.busy is None and bool(w.homed) and bool(w.calibrated) and not w.fault)
        self.btn_run.setEnabled(ok and not self.running)
        if self.running:
            self.btn_run.setToolTip("Barrido en curso")
        elif not w.link.is_open:
            self.btn_run.setToolTip("No hay conexión con la placa")
        elif not (w.homed and w.calibrated):
            self.btn_run.setToolTip("Primero «Calibrar e iniciar» (M56) en la pestaña Control:\n"
                                    "M57 necesita saber dónde está el carro para no salirse del recorrido")
        else:
            self.btn_run.setToolTip("Corre M57 … R1 ida y vuelta y rehabilita el lazo (M17) al terminar")
        self.btn_png.setEnabled(self.path is not None and self.proc is None)

    # --- barrido ----------------------------------------------------------------------
    def start(self):
        w = self.win
        if not self.btn_run.isEnabled():
            return
        revs = self.spin_revs.value()
        v = self.spin_sweep_speed.value()
        dist = revs * PITCH_MM
        pos = w.last[History.POS] if (w.last is not None and w.telemetry_alive()) else w.dest
        if pos is None:
            w.message("No se sabe dónde está el carro (sin telemetría): esperar un momento", AMBER)
            return
        if pos + dist <= AXIS_MAX_MM - 0.5:
            legs = [dist, -dist]
        elif pos - dist >= AXIS_MIN_MM + 0.5:
            legs = [-dist, dist]
        else:
            w.message("No entra el barrido en el recorrido", RED)
            return
        dur = 2 * dist / v
        ans = QtWidgets.QMessageBox.question(
            self, "Barrido del encoder",
            f"El motor gira {revs} vuelta{'s' if revs > 1 else ''} en lazo abierto y vuelve "
            f"(el carro va de {pos:.2f} a {pos + legs[0]:.2f} mm y regresa), a {v:g} mm/s: unos {dur + 2:.0f} s.\n\n"
            "Durante el barrido el lazo cerrado está apagado y la placa no atiende otros comandos "
            "(ni «Apagar motor»). Al terminar se rehabilita el lazo (M17) donde quedó el carro.\n\n"
            "Asegurate de que el recorrido esté libre. ¿Empezar?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ans != QtWidgets.QMessageBox.Yes:
            return
        self.rows, self.meta, self.end_lines = [], {}, []
        self.legs, self.leg_i, self.speed = legs, 0, v
        self.running = True
        self.path = None
        self.list.blockSignals(True)
        self.list.clearSelection()
        self.list.setCurrentRow(-1)
        self.list.blockSignals(False)
        self.lbl_src.setText("Barrido en curso…")
        self.lbl_verdict.hide()
        w.busy = self.BUSY_LABEL
        w.busy_since = time.monotonic()
        w.busy_detail = ""
        w.update_controls()
        self._send_leg()

    def _send_leg(self):
        d = self.legs[self.leg_i]
        text = f"M57 X{d:.4f} F{self.speed:g} P{SWEEP_DECIMATION} R1"
        self.win.log_line(f"> {text}   (pestaña Encoder, pasada {self.leg_i + 1} de {len(self.legs)})", "cmd")
        self._seen = 0
        self.leg_field0 = None
        self._leg_rows0 = len(self.rows)
        self._cmd = self.win.link.send(text, silent=True, on_done=self._leg_done)

    def _consume(self, cmd):
        lines = cmd.lines
        n = len(lines)
        for line in lines[self._seen:n]:
            r = ea.parse_line(line)
            if r is not None:
                self.rows.append(r)
                if self.leg_field0 is None:
                    self.leg_field0 = r[2]
            elif line.startswith("META"):
                if not self.meta:
                    self.meta = ea.parse_meta(line)
                    self.win.log_line(line, "reply")
            elif line.startswith("END"):
                self.end_lines.append(line)
                self.win.log_line(line, "reply")
            elif line.startswith("error"):
                self.win.log_line(line, "error")
        if n > self._seen:
            self._live_dirty = True
        self._seen = n

    def _leg_progress(self):
        if self.leg_field0 is None or not self.rows or not self.meta:
            return 0.0
        pp = self.meta.get("pole_pairs", MOTOR_POLE_PAIRS)
        total = abs(self.legs[self.leg_i]) / PITCH_MM * 2 * math.pi * pp
        return clamp(abs(self.rows[-1][2] - self.leg_field0) / total, 0.0, 1.0)

    def _tick_live(self):
        if not self.running or self._cmd is None:
            return
        self._consume(self._cmd)
        frac = (self.leg_i + self._leg_progress()) / len(self.legs)
        self.progress.setValue(int(frac * 1000))
        sense = "ida" if self.leg_i == 0 else "vuelta"
        self.lbl_run.setText(f"Pasada {self.leg_i + 1} de {len(self.legs)} ({sense}) · {len(self.rows)} muestras")
        self.win.busy_detail = f"{sense}, {100 * frac:.0f} %"
        if self._live_dirty and len(self.rows) > 50:
            self._live_dirty = False
            self.T = ea.columns(self._make_table(np.array(self.rows, dtype=float)))
            self._draw_sweep()

    def _leg_done(self, cmd):
        self._consume(cmd)
        got = len(self.rows) - self._leg_rows0
        if cmd.ok is not True or got == 0:
            if cmd.ok is None:
                why = ("se perdió la conexión" if not self.win.link.is_open
                       else f"la placa no respondió en {cmd.timeout:g} s")
            else:
                why = cmd.lines[-1] if cmd.lines else "sin respuesta"
            if cmd.ok and got == 0:
                why = "la placa respondió ok pero sin muestras: ¿el firmware grabado tiene M57 R1?"
            self._finish(why)
            return
        self.leg_i += 1
        if self.leg_i < len(self.legs):
            self._send_leg()
        else:
            self._finish()

    def _finish(self, error=None):
        w = self.win
        self.running = False
        self._cmd = None
        w.busy = None
        if w.link.is_open:
            w.link.send("M17", on_done=w._enable_done)    # el M57 deja el lazo cerrado apagado
        self.progress.setValue(1000 if error is None else self.progress.value())
        if len(self.rows) >= 100:
            table = self._make_table(np.array(self.rows, dtype=float))
            now = datetime.datetime.now()
            folder = os.path.join(REPO, "mediciones", now.strftime("%Y-%m-%d"))
            os.makedirs(folder, exist_ok=True)
            path = os.path.join(folder, now.strftime("encoder_sweep_%Y%m%d_%H%M%S.csv"))
            ea.save_csv(table, path)
            with open(path, "a") as f:
                # al final: np.loadtxt y genfromtxt(names=True) saltean los comentarios
                f.write(f"# barrido desde tools/interfaz.py (M57 R1), {now:%Y-%m-%d %H:%M:%S}; "
                        f"motor_deg y x_mm cuentan desde el 0 del eje (home)\n")
                f.write("# META," + ",".join(f"{a}={b}" for a, b in self.meta.items()) + "\n")
                for line in self.end_lines:
                    f.write(f"# {line}\n")
            self.path = path
            self._show(table)
            self.refresh_list(select=path)
            self.lbl_run.setText(f"{len(self.rows)} muestras · guardado en {os.path.relpath(path, REPO)}")
            w.log_line(f"— barrido del encoder guardado: {path} —", "info")
        if error:
            self.lbl_run.setText(f"Barrido cortado: {error}")
            w.message(f"Barrido del encoder: {error}", RED)
            QtWidgets.QMessageBox.warning(self, "Barrido del encoder", f"El barrido no terminó:\n\n{error}")
        else:
            w.message("Barrido del encoder terminado; el lazo se rehabilita donde quedó el carro", GREEN)
        w.update_controls()

    # --- cargar mediciones --------------------------------------------------------------
    def refresh_list(self, select=None):
        cur = select or self.path
        self.list.blockSignals(True)
        self.list.clear()
        for p in _sweep_csvs():
            it = QtWidgets.QListWidgetItem(_sweep_label(p))
            it.setData(Qt.UserRole, p)
            it.setToolTip(os.path.relpath(p, REPO))
            self.list.addItem(it)
            if p == cur:
                self.list.setCurrentItem(it)
        self.list.blockSignals(False)

    def _list_picked(self, item, _prev=None):
        if item is not None and not self.running:
            self.load(item.data(Qt.UserRole))

    def open_dialog(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Abrir barrido del encoder",
                                                        os.path.join(REPO, "mediciones"), "CSV (*.csv)")
        if path:
            self.load(path)

    def load(self, path):
        if self.running:
            return
        try:
            table = ea.load_csv(path)
            if table.shape[1] != len(ea.COLS):
                raise ValueError(f"tiene {table.shape[1]} columnas y un barrido tiene {len(ea.COLS)}")
            with open(path, errors="replace") as f:
                self.from_home = "desde el 0 del eje" in f.read()
        except (OSError, ValueError) as e:
            self.win.message(f"No se pudo leer {os.path.basename(path)}: {e}", RED)
            return
        self.path = path
        self._show(table)

    # --- análisis y gráficos -----------------------------------------------------------
    def _make_table(self, raw):
        pp = self.meta.get("pole_pairs", MOTOR_POLE_PAIRS)
        ref = self.meta.get("field_zero")
        self.from_home = ref is not None
        return ea.build_table(raw, pp, ea.CPR, WHEEL_MAGNETS, WHEEL_PITCH_MM, field_ref=ref)[0]

    def _show(self, table):
        self.table = table
        self.T = ea.columns(table)
        self.stats = ea.summary(table, WHEEL_MAGNETS / 2)
        self.windows = ea.window_stats(table, WHEEL_MAGNETS, WHEEL_PITCH_MM, 2)
        name = os.path.relpath(self.path, REPO) if self.path else "barrido sin guardar"
        ref = "posición contada desde el home (0 mm)" if self.from_home else \
            "posición contada desde el inicio del barrido (imán 0 = el que estaba frente al sensor)"
        self.lbl_src.setText(f"{name}<br><span style='color:{MUTED}'>{ref}</span>")
        self._fill_results()
        self._draw_sweep()
        self._draw_overview()
        n_win = len(self.windows) // 2
        self.win_combo.blockSignals(True)
        self.win_combo.clear()
        for k in range(n_win):
            r = self.windows[2 * k]
            self.win_combo.addItem(f"{k + 1} de {n_win} · imanes {r['mag0']}–{r['mag1']} · "
                                   f"{r['deg0']:.0f}–{r['deg1']:.0f}° de la vuelta")
        self.win_combo.blockSignals(False)
        self.select_window(min(self.cur_win, n_win - 1))
        self._draw_phase()
        self._draw_status()
        self.update_controls()

    def _fill_results(self):
        s = self.stats
        good, bad, warn = (f"color:{GREEN}", f"color:{RED}", f"color:{AMBER}")
        ppr_ok = abs(s["periods_per_rev"] - WHEEL_MAGNETS / 2) < 0.2
        self.res["ppr"].setText(f"{'✓' if ppr_ok else '✗'} {s['periods_per_rev']:.2f}  (esperado {WHEEL_MAGNETS // 2})")
        self.res["ppr"].setStyleSheet(good if ppr_ok else bad)
        self.res["jumps"].setText(f"{'✓' if s['jumps'] == 0 else '✗'} {s['jumps']}")
        self.res["jumps"].setStyleSheet(good if s["jumps"] == 0 else bad)
        self.res["weak"].setText(f"{s['weak_pct']:.1f} %  ({s['weak_n']} muestras)")
        self.res["weak"].setStyleSheet(good if s["weak_n"] == 0 else warn)
        for k in ("ida", "vuelta"):
            e = s[k]
            self.res[k].setText("—" if e is None else f"RMS {e['rms']:.1f}° · máx {e['max']:.0f}°")
            if e is not None:
                self.res[k].setToolTip(f"RMS {e['rms']:.2f}° de encoder = {e['rms_shaft']:.3f}° de eje "
                                       f"(1 período del encoder = 360°, 30 períodos por vuelta)")
            self.res[k].setStyleSheet(f"color:{INK}")
        bits_ok = s["overspeed_n"] == 0 and s["crc_n"] == 0 and s["undervolt_n"] == 0
        self.res["bits"].setText(f"{s['overspeed_n']} / {s['crc_n']} / {s['undervolt_n']}")
        self.res["bits"].setStyleSheet(good if bits_ok else warn)
        self.res["n"].setText(f"{s['n']} en {s['duration_s']:.1f} s · {s['revs']:.2f} vueltas")
        self.res["n"].setStyleSheet(f"color:{INK}")

        problems = []
        if not ppr_ok:
            problems.append(f"la lectura no da {WHEEL_MAGNETS // 2} períodos por vuelta")
        if s["jumps"]:
            problems.append(f"{s['jumps']} saltos (ver «Saltos y estado» para ubicarlos)")
        if s["crc_n"]:
            problems.append(f"{s['crc_n']} errores de CRC (cableado/SPI)")
        if problems:
            text = ("✗ " + "; ".join(problems) + ". En lazo abierto un salto también puede ser el motor "
                    "patinando en un punto duro: comparar ida y vuelta y la posición donde aparece.")
            self.lbl_verdict.setStyleSheet(f"background:#fbe3e3; color:{RED}; padding:6px 8px; border-radius:6px;"
                                           "font-weight:600")
        else:
            text = (f"✓ La lectura sigue a la rueda en todo el barrido: {s['periods_per_rev']:.2f} períodos "
                    f"por vuelta, sin saltos.")
            self.lbl_verdict.setStyleSheet("background:#e2f3e8; color:#1f6e3f; padding:6px 8px; border-radius:6px;"
                                           "font-weight:600")
        self.lbl_verdict.setText(text)
        self.lbl_verdict.show()

    def _draw_sweep(self):
        T = self.T
        x = T["motor_deg"] / 360.0
        y = ea.wrap180(T["enc_deg"])
        fwd = T["dir"] > 0
        bwd = ~fwd
        wk = T["weak"] > 0
        self.c_ang[0].setData(x[fwd], y[fwd])
        self.c_ang[1].setData(x[bwd], y[bwd])
        self.c_ang[2].setData(x[wk], y[wk])
        self.c_errx[0].setData(x[fwd], T["error_deg"][fwd])
        self.c_errx[1].setData(x[bwd], T["error_deg"][bwd])
        jm = np.concatenate([[False], ea.jumps(T)[2]]) if len(x) > 1 else np.zeros(len(x), bool)
        self.c_errx[2].setData(x[jm], T["error_deg"][jm])
        for ln in self.rev_lines:
            self.p_ang.removeItem(ln)
        self.rev_lines = []
        if len(x):
            for k in range(int(math.ceil(np.nanmin(x))), int(math.floor(np.nanmax(x))) + 1):
                ln = pg.InfiniteLine(pos=k, angle=90, pen=pg.mkPen(INK2, width=1, style=Qt.DotLine))
                self.p_ang.addItem(ln)
                self.rev_lines.append(ln)
        unit = "posición del motor [vueltas desde el home]" if self.from_home else \
            "posición del motor [vueltas desde el inicio]"
        self.p_errx.setLabel("bottom", unit)
        note = KEY_DIRS + KEY_WEAK + " &nbsp;· punteado: cada vuelta del motor"
        if self.from_home and len(x):
            a, b = (round(v * PITCH_MM, 2) + 0.0 for v in (np.nanmin(x), np.nanmax(x)))
            note += f" · carro de {a:.2f} a {b:.2f} mm"
        MainWindow._set_title(self.p_ang, "Ángulo crudo del encoder", note)
        for p in (self.p_ang, self.p_errx):
            p.getViewBox().enableAutoRange(x=True)

    def _bars(self, p, store, x, h, width, color, name=None):
        it = pg.BarGraphItem(x=x, height=h, width=width, brush=pg.mkBrush(color), pen=pg.mkPen(SURFACE, width=1))
        p.addItem(it)
        store.append(it)
        if name is not None and p.legend is not None:
            p.legend.addItem(it, name)
        return it

    def _draw_overview(self):
        for it in self.ov_items:
            for p in (self.p_ov_rms, self.p_ov_weak):
                if it.scene() is not None and it in p.items:
                    p.removeItem(it)
        self.ov_items = []
        def key_rms(flipped):
            return key + (f" &nbsp;<span style='color:{RED}'>▼</span> hay muestras corridas ±180°" if flipped else "")
        key = KEY_DIRS
        MainWindow._set_title(self.p_ov_weak, "Campo débil por ventana", key)
        n_win = len(self.windows) // 2
        idx = np.arange(n_win)
        for stat, p in (("rms", self.p_ov_rms), ("weak_pct", self.p_ov_weak)):
            for s, off, col, name in ((+1, -0.2, BLUE, "ida"), (-1, 0.2, ORANGE, "vuelta")):
                vals = np.array([r[stat] for r in self.windows if r["dir"] == s], dtype=float)
                self._bars(p, self.ov_items, idx + off, np.nan_to_num(vals), 0.38, col)
            flips = np.array([r["flips"] for r in self.windows if r["dir"] == +1]) + \
                np.array([r["flips"] for r in self.windows if r["dir"] == -1])
            if stat == "rms" and flips.any():
                top = np.nanmax([r["rms"] for r in self.windows]) if self.windows else 1
                sc = pg.ScatterPlotItem(idx[flips > 0], np.full((flips > 0).sum(), top * 1.08), symbol="t",
                                        size=9, brush=pg.mkBrush(RED), pen=None)
                p.addItem(sc)
                self.ov_items.append(sc)
            if stat == "rms":
                MainWindow._set_title(p, "Error RMS por ventana", key_rms(flips.any()))
        ticks = [(k, f"{self.windows[2 * k]['mag0']}–{self.windows[2 * k]['mag1']}") for k in range(n_win)]
        self.p_ov_weak.getAxis("bottom").setTicks([ticks])
        self.p_ov_rms.getAxis("bottom").setTicks([ticks])
        self.p_ov_rms.getAxis("bottom").setStyle(showValues=False)
        self.p_ov_rms.setXRange(-0.6, n_win - 0.4, padding=0)
        for p in (self.p_ov_rms, self.p_ov_weak):
            p.enableAutoRange(y=True)

    def _ov_clicked(self, ev):
        if self.windows is None:
            return
        for p in (self.p_ov_rms, self.p_ov_weak):
            if p.sceneBoundingRect().contains(ev.scenePos()):
                k = int(round(p.getViewBox().mapSceneToView(ev.scenePos()).x()))
                if 0 <= k < self.win_combo.count():
                    self.win_combo.setCurrentIndex(k)
                return

    def step_window(self, d):
        n = self.win_combo.count()
        if n:
            self.win_combo.setCurrentIndex((self.win_combo.currentIndex() + d) % n)

    def select_window(self, k):
        if self.windows is None or k < 0:
            return
        if self.win_combo.currentIndex() != k:
            self.win_combo.blockSignals(True)
            self.win_combo.setCurrentIndex(k)
            self.win_combo.blockSignals(False)
        self.cur_win = k
        for reg in self.ov_regions:
            reg.setRegion((k - 0.5, k + 0.5))
        T = self.T
        steady = ea.steady_mask(T)
        circ, win_mm, _ = ea.window_geometry(WHEEL_MAGNETS, WHEEL_PITCH_MM, 2)
        xm = (T["motor_deg"] % 360.0) / 360.0 * circ / WHEEL_PITCH_MM      # en imanes
        x0, x1 = k * win_mm / WHEEL_PITCH_MM, (k + 1) * win_mm / WHEEL_PITCH_MM
        enc = ea.wrap180(T["enc_deg"])
        ideal = ea.wrap180(T["ideal_deg"])
        passes = _pass_index(T)
        for p, s, name in ((self.p_win[0], +1, "Ida"), (self.p_win[1], -1, "Vuelta")):
            p.clear()
            p.legend.clear()
            sel = steady & (T["dir"] == s) & (xm >= x0) & (xm < x1)
            for kk in range(int(round(x0)), int(round(x1)) + 1):
                p.addLine(x=kk, pen=pg.mkPen(GRID if kk % 2 else MUTED, width=1))
            for r in sorted(set(passes[sel])):
                m = sel & (passes == r)
                o = np.argsort(xm[m])
                col = PASS_COLORS[min(r, len(PASS_COLORS) - 1)]
                first = r == min(set(passes[sel]))
                c = QtGui.QColor(col)
                c.setAlpha(90 if first else 255)
                p.plot(xm[m][o], enc[m][o], pen=None, symbol="o", symbolSize=7 if first else 3, symbolPen=None,
                       symbolBrush=c, name=f"pasada {r + 1}")
                if first:
                    p.plot(xm[m][o], ea.nan_at_wraps(ideal[m][o]), pen=pg.mkPen(INK2, width=1, style=Qt.DashLine),
                           name="ideal", connect="finite")
            for mask, col, sym, label in ((sel & (T["weak"] > 0), RED, "x", "campo débil"),
                                          (sel & (T["overspeed"] > 0), AMBER, "+", "sobrevelocidad")):
                if mask.any():
                    p.plot(xm[mask], enc[mask], pen=None, symbol=sym, symbolSize=8, symbolPen=pg.mkPen(col, width=1.5),
                           symbolBrush=None, name=label)
            r = self.windows[2 * k + (0 if s > 0 else 1)]
            MainWindow._set_title(p, name, f"débil {r['weak_pct']:.0f} % · error RMS {r['rms']:.0f}° · "
                                           f"corridas {r['flips']}")
            p.setXRange(x0, x1, padding=0.01)
            p.setYRange(-190, 190, padding=0)

    def _draw_phase(self):
        T = self.T
        fwd = T["dir"] > 0
        self.c_phase[0].setData(T["ideal_deg"][fwd], T["error_deg"][fwd])
        self.c_phase[1].setData(T["ideal_deg"][~fwd], T["error_deg"][~fwd])
        for it in self.phase_items:
            for p in (self.p_phase_bits, self.p_mag):
                if it in p.items:
                    p.removeItem(it)
        self.phase_items = []
        self.p_phase_bits.legend.clear()
        bins = np.linspace(0, 360, 37)
        idx = np.clip(np.digitize(T["ideal_deg"], bins) - 1, 0, 35)
        fw = [T["weak"][idx == k].mean() * 100 if np.any(idx == k) else 0 for k in range(36)]
        fo = [T["overspeed"][idx == k].mean() * 100 if np.any(idx == k) else 0 for k in range(36)]
        self._bars(self.p_phase_bits, self.phase_items, bins[:-1] + 2.75, fw, 4.5, RED, "campo débil")
        self._bars(self.p_phase_bits, self.phase_items, bins[:-1] + 7.25, fo, 4.5, AMBER, "sobrevelocidad")
        mags = (T["motor_deg"] % 360.0) / 360.0 * WHEEL_MAGNETS
        idx = np.clip(np.floor(mags).astype(int), 0, WHEEL_MAGNETS - 1)
        frac = [T["weak"][idx == k].mean() * 100 if np.any(idx == k) else 0 for k in range(WHEEL_MAGNETS)]
        self._bars(self.p_mag, self.phase_items, np.arange(WHEEL_MAGNETS) + 0.5, frac, 0.8, RED)
        ref = "desde el home" if self.from_home else "desde el inicio del barrido"
        MainWindow._set_title(self.p_mag, "Campo débil a lo largo de la vuelta (1 barra = 1 imán)", f"imán 0: {ref}")
        self.p_mag.setXRange(0, WHEEL_MAGNETS, padding=0.01)
        for p in (self.p_phase, self.p_phase_bits, self.p_mag):
            p.enableAutoRange()

    def _draw_status(self):
        T = self.T
        t, dev, mask = ea.jumps(T)
        self.c_jump.setData(t, dev)
        self.c_jump_x.setData(t[mask], dev[mask])
        for k, c in enumerate(["overspeed", "weak", "undervolt", "crc_err"]):
            m = T[c] > 0
            self.c_bits[k].setData(T["t_s"][m], np.full(int(m.sum()), k))
        self.p_jump.enableAutoRange()
        self.p_bits.setXRange(T["t_s"][0], T["t_s"][-1], padding=0.01)

    # --- PNG con los scripts ----------------------------------------------------------
    def export_png(self):
        if not self.path or self.proc is not None:
            return
        tools = os.path.join(REPO, "tools")
        self._jobs = [[os.path.join(tools, "encoder_sweep.py"), "--from-csv", self.path],
                      [os.path.join(tools, "plot_periods.py"), self.path]]
        self._job_out = []
        self.win.message("Generando los PNG con encoder_sweep.py y plot_periods.py…")
        self._next_job()

    def _next_job(self):
        if not self._jobs:
            self.proc = None
            pngs = [l.split(" en ", 1)[1] for l in self._job_out if l.startswith("grafico")]
            self.win.message("PNG: " + " · ".join(os.path.relpath(p, REPO) for p in pngs) if pngs
                             else "No se generaron PNG (ver la consola)", GREEN if pngs else RED)
            for l in self._job_out[-4:]:
                self.win.log_line(l, "info")
            self.update_controls()
            return
        args = self._jobs.pop(0)
        self.proc = QtCore.QProcess(self)
        self.proc.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.proc.finished.connect(self._job_done)
        self.proc.start(sys.executable, args)
        self.update_controls()

    def _job_done(self, *_):
        out = bytes(self.proc.readAll()).decode(errors="replace")
        self._job_out += [l.strip() for l in out.splitlines() if l.strip()]
        self._next_job()


# ======================================================================================
# Ventana principal
# ======================================================================================

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, port=None):
        super().__init__()
        self.setWindowTitle("μMachining — control del eje")
        self.link = Link()
        self.link.console.connect(self.on_console)
        self.link.done.connect(self.on_done)
        self.link.lost.connect(self.on_lost)
        self.hist = History(int(HISTORY_S * 2000 / TELEMETRY_DECIMATION))

        # estado del eje (None: no se sabe)
        self.homed = None
        self.calibrated = None
        self.loop_on = None
        self.fault = None
        self.dest = None               # destino del último G1 (lo que pidió la interfaz o M119)
        self.moves_pending = 0         # G1 mandados sin respuesta (M119 no pisa el destino)
        self.fw_max_speed = SPEED_DEFAULT
        self.busy = None               # texto mientras corre un comando largo
        self.busy_since = 0.0
        self.busy_detail = ""
        self.last_tel_time = 0.0       # time.monotonic() del último paquete
        self.rate = 0.0
        self._rate_count = 0
        self._rate_t0 = time.monotonic()
        self.last = None               # última fila: t, target, pos, vel, out, status, flags
        self.cmd_vel = 0.0             # velocidad del objetivo [mm/s]
        self.rec_file = None
        self.rec_path = None
        self.paused = False
        self._spin_init = False
        self.user_relative = False     # el usuario mandó G91 por la consola
        self._win_err = None
        self._pos_note = ""            # referencia del gráfico de posición cuando rotula en µm

        self._build_ui()
        self.keyfilter = KeyFilter(self)
        QtWidgets.QApplication.instance().installEventFilter(self.keyfilter)

        self.plot_timer = QtCore.QTimer(self, timeout=self.tick_plots)
        self.plot_timer.start(40)
        self.ui_timer = QtCore.QTimer(self, timeout=self.tick_ui)
        self.ui_timer.start(100)
        self.poll_timer = QtCore.QTimer(self, timeout=self.poll_status)
        self.poll_timer.start(1500)

        self.refresh_ports(prefer=port)
        self.update_controls()
        if self.port_combo.currentText():
            QtCore.QTimer.singleShot(200, self.connect_port)

    # ----------------------------------------------------------------------------------
    # UI
    # ----------------------------------------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(10, 8, 10, 4)
        root.setSpacing(8)

        # --- barra superior: conexión, estado, parada ---
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(8)
        self.port_combo = QtWidgets.QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setMinimumWidth(300)
        self.port_combo.setToolTip("Puerto serie de la placa")
        btn_refresh = QtWidgets.QToolButton(text="↻", toolTip="Volver a buscar puertos")
        btn_refresh.clicked.connect(lambda: self.refresh_ports())
        self.btn_connect = QtWidgets.QPushButton("Conectar")
        self.btn_connect.setMinimumWidth(110)
        self.btn_connect.clicked.connect(self.toggle_connection)
        top.addWidget(QtWidgets.QLabel("Puerto:"))
        top.addWidget(self.port_combo)
        top.addWidget(btn_refresh)
        top.addWidget(self.btn_connect)
        top.addSpacing(16)
        self.chip_conn = Chip("Desconectado", "Conexión con la placa")
        self.chip_cal = Chip("Calibración", "Verde: el eje está calibrado (M56)")
        self.chip_home = Chip("Homing", "Verde: el eje sabe dónde está el 0")
        self.chip_loop = Chip("Lazo", "Verde: lazo cerrado activo, el eje sigue los comandos")
        for c in (self.chip_conn, self.chip_cal, self.chip_home, self.chip_loop):
            top.addWidget(c)
        top.addStretch(1)
        self.btn_off = QtWidgets.QPushButton("■  APAGAR MOTOR  (Esc)")
        self.btn_off.setObjectName("danger")
        self.btn_off.setToolTip("Corta la corriente del motor (M18). El carro queda libre donde está.\n"
                                "Para volver a mover: «Habilitar lazo».")
        self.btn_off.clicked.connect(self.motor_off)
        top.addWidget(self.btn_off)
        root.addLayout(top)

        # --- aviso (falla, lazo apagado, calibración en curso) ---
        self.banner = QtWidgets.QLabel()
        self.banner.setWordWrap(True)
        self.banner.setObjectName("banner")
        self.banner.hide()
        root.addWidget(self.banner)

        split = QtWidgets.QSplitter(Qt.Horizontal)
        split.addWidget(self._build_controls())
        split.addWidget(self._build_live())
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([400, 1100])
        # la barra de arriba (conexión, estado, APAGAR MOTOR) y el aviso quedan fuera de las pestañas
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setObjectName("mainTabs")
        self.tabs.addTab(split, "Control")
        self.enc_tab = EncoderTab(self)
        self.tabs.addTab(self.enc_tab, "Encoder")
        self.tabs.setTabToolTip(1, "Barrido del encoder en lazo abierto (como encoder_sweep.py y plot_periods.py)")
        root.addWidget(self.tabs, 1)

        # --- barra de estado ---
        self.status_msg = QtWidgets.QLabel()
        self.status_tel = QtWidgets.QLabel()
        self.status_tel.setMinimumWidth(260)
        self.status_rec = QtWidgets.QLabel()
        self.statusBar().addWidget(self.status_msg, 1)
        self.statusBar().addPermanentWidget(self.status_rec)
        self.statusBar().addPermanentWidget(self.status_tel)

    def _group(self, title):
        box = QtWidgets.QGroupBox(title)
        lay = QtWidgets.QVBoxLayout(box)
        lay.setSpacing(6)
        return box, lay

    def _build_controls(self):
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(360)
        panel.setMaximumWidth(470)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 4, 0)
        lay.setSpacing(8)

        # 1. puesta en marcha
        box, b = self._group("1 · Puesta en marcha")
        self.btn_cal = QtWidgets.QPushButton("Calibrar e iniciar  (M56, ~3 min)")
        self.btn_cal.setToolTip("Homing contra el tope de 0 mm, calibración del encoder y reaprendizaje del\n"
                                "anti-cogging (el carro va y vuelve dos veces entre 0 y 50 mm).\n"
                                "Hacerlo cada vez que se enciende la placa.")
        self.btn_cal.clicked.connect(self.calibrate)
        self.btn_enable = QtWidgets.QPushButton("Habilitar lazo  (M17)")
        self.btn_enable.setToolTip("Prende el motor con el lazo cerrado, quieto donde está el carro.\n"
                                   "Sirve también para rearmar después de un corte por falla o de apagar el motor.")
        self.btn_enable.clicked.connect(self.enable_loop)
        b.addWidget(self.btn_cal)
        b.addWidget(self.btn_enable)
        lay.addWidget(box)

        # 2. ir a una posición
        box, b = self._group("2 · Ir a una posición")
        form = QtWidgets.QGridLayout()
        form.setHorizontalSpacing(8)
        self.spin_dest = SpinBox(AXIS_MIN_MM, AXIS_MAX_MM, POS_DECIMALS, 1.0, " mm")
        self.spin_dest.setToolTip("Destino absoluto (0 a 255 mm). Enter = Ir.\nTambién se elige con un clic en el carril.")
        self.spin_dest.lineEdit().returnPressed.connect(self.go)
        self.spin_speed = SpinBox(0.01, SPEED_MAX, 2, 0.1, " mm/s")
        self.spin_speed.setValue(SPEED_DEFAULT)
        self.spin_speed.setToolTip(f"Velocidad para Ir, los pasos y el movimiento continuo.\n"
                                   f"Hasta {SPEED_SAFE:g} mm/s: precisión completa (anti-cogging aprendido a 2 mm/s).\n"
                                   f"Más rápido el anti-cogging se apaga y el error crece; a 4 mm/s falta torque.")
        self.spin_speed.valueChanged.connect(self._speed_changed)
        self.lbl_speed_warn = QtWidgets.QLabel()
        self.lbl_speed_warn.setWordWrap(True)
        self.lbl_speed_warn.setStyleSheet(f"color:{AMBER}")
        self.btn_go = QtWidgets.QPushButton("Ir  ▶")
        self.btn_go.setObjectName("primary")
        self.btn_go.clicked.connect(self.go)
        form.addWidget(QtWidgets.QLabel("Destino"), 0, 0)
        form.addWidget(self.spin_dest, 0, 1)
        form.addWidget(QtWidgets.QLabel("Velocidad"), 1, 0)
        form.addWidget(self.spin_speed, 1, 1)
        form.addWidget(self.btn_go, 0, 2, 2, 1)
        form.setColumnStretch(1, 1)
        b.addLayout(form)
        b.addWidget(self.lbl_speed_warn)
        presets = QtWidgets.QHBoxLayout()
        presets.setSpacing(4)
        self.preset_btns = []
        for label, mm in (("0", 0.0), ("50", 50.0), ("Centro", 127.5), ("200", 200.0), ("255", 255.0)):
            btn = QtWidgets.QPushButton(label)
            btn.setToolTip(f"Ir a {mm:g} mm")
            btn.clicked.connect(lambda _, mm=mm: self.move_to(mm))
            presets.addWidget(btn)
            self.preset_btns.append(btn)
        b.addLayout(presets)
        lay.addWidget(box)

        # 3. pasos y movimiento continuo
        box, b = self._group("3 · Mover a mano")
        steps = QtWidgets.QHBoxLayout()
        steps.setSpacing(4)
        steps.addWidget(QtWidgets.QLabel("Paso:"))
        self.step_exp = 0                                  # paso = 10^step_exp mm
        self.btn_step_down = QtWidgets.QPushButton("÷10")
        self.btn_step_up = QtWidgets.QPushButton("×10")
        self.lbl_step = QtWidgets.QLabel(alignment=Qt.AlignCenter)
        self.lbl_step.setObjectName("stepValue")
        self.lbl_step.setMinimumWidth(QtGui.QFontMetrics(self.lbl_step.font()).horizontalAdvance("0000 µm") * 2)
        for btn, k in ((self.btn_step_down, -1), (self.btn_step_up, +1)):
            btn.setToolTip(f"Paso {btn.text()} (teclas ↑ ↓), de {fmt_len(STEP_EXP_MIN)} a {fmt_len(STEP_EXP_MAX)}")
            btn.clicked.connect(lambda _, k=k: self.step_scale(k))
        steps.addWidget(self.btn_step_down)
        steps.addWidget(self.lbl_step, 1)
        steps.addWidget(self.btn_step_up)
        b.addLayout(steps)
        self.step_scale(0)
        jog = QtWidgets.QHBoxLayout()
        self.btn_minus = QtWidgets.QPushButton("◀  − paso")
        self.btn_plus = QtWidgets.QPushButton("+ paso  ▶")
        for btn in (self.btn_minus, self.btn_plus):
            btn.setObjectName("big")
            btn.setToolTip("Mueve un paso desde el destino actual (teclas ← →)")
        self.btn_minus.clicked.connect(lambda: self.jog(-1))
        self.btn_plus.clicked.connect(lambda: self.jog(+1))
        jog.addWidget(self.btn_minus)
        jog.addWidget(self.btn_plus)
        b.addLayout(jog)
        hold = QtWidgets.QHBoxLayout()
        self.btn_hold_minus = QtWidgets.QPushButton("◀◀  mantener")
        self.btn_hold_plus = QtWidgets.QPushButton("mantener  ▶▶")
        for btn, sign in ((self.btn_hold_minus, -1), (self.btn_hold_plus, +1)):
            btn.setToolTip("Se mueve mientras se mantiene apretado y frena al soltar")
            btn.pressed.connect(lambda sign=sign: self.hold_start(sign))
            btn.released.connect(self.hold_stop)
            hold.addWidget(btn)
        b.addLayout(hold)
        self.btn_stop = QtWidgets.QPushButton("■  Detener  (Espacio)")
        self.btn_stop.setObjectName("stop")
        self.btn_stop.setToolTip("Frena el movimiento y deja el carro quieto con el lazo activo")
        self.btn_stop.clicked.connect(self.stop_motion)
        b.addWidget(self.btn_stop)
        lay.addWidget(box)

        # consola
        box, b = self._group("Consola G-code")
        self.console_view = QtWidgets.QPlainTextEdit(readOnly=True)
        self.console_view.setMaximumBlockCount(5000)
        self.console_view.setObjectName("console")
        self.gcode = GcodeLine()
        self.gcode.returnPressed.connect(self.send_typed)
        b.addWidget(self.console_view, 1)
        b.addWidget(self.gcode)
        extra = QtWidgets.QHBoxLayout()
        extra.setSpacing(4)
        for label, cmd, tip in (("Estado", "M119", "M119: estado completo del eje"),
                                ("Anti-cogging", "M59", "M59: estado de la compensación de retención"),
                                ("Traza corte", "M121", "M121: los ~2 s previos al último corte por falla")):
            btn = QtWidgets.QPushButton(label, toolTip=tip, objectName="seg")
            btn.clicked.connect(lambda _, c=cmd: self.send_console(c))
            extra.addWidget(btn)
        btn_clear = QtWidgets.QPushButton("Limpiar", toolTip="Borrar la consola", objectName="seg")
        btn_clear.clicked.connect(self.console_view.clear)
        extra.addWidget(btn_clear)
        b.addLayout(extra)
        lay.addWidget(box, 1)
        return panel

    def _build_live(self):
        panel = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(4, 0, 0, 0)
        lay.setSpacing(8)

        # valores grandes
        grid = QtWidgets.QGridLayout()
        grid.setSpacing(8)
        self.r_pos = Readout("Posición", "mm", "Posición medida por el encoder")
        self.r_dest = Readout("Destino", "mm", "A dónde va el carro (último comando)")
        self.r_err = Readout("Error de seguimiento", "µm",
                             "Objetivo del perfil menos posición medida (promedio de 0,1 s).\n"
                             "El firmware corta si pasa de 1 mm durante 0,5 s.")
        self.r_vel = Readout("Velocidad", "mm/s", "Velocidad medida (promedio de 0,1 s)")
        grid.addWidget(self.r_pos, 0, 0)
        grid.addWidget(self.r_dest, 0, 1)
        grid.addWidget(self.r_err, 0, 2)
        grid.addWidget(self.r_vel, 0, 3)

        torque = QtWidgets.QFrame()
        torque.setObjectName("readout")
        tl = QtWidgets.QVBoxLayout(torque)
        tl.setContentsMargins(10, 6, 10, 6)
        tl.setSpacing(2)
        tl.addWidget(QtWidgets.QLabel(f"Uso de torque <span style='color:{MUTED}'>[salida del lazo]</span>",
                                      objectName="readoutTitle"))
        self.bar_out = QtWidgets.QProgressBar()
        self.bar_out.setRange(0, 100)
        self.bar_out.setTextVisible(False)
        self.bar_out.setFixedHeight(14)
        self.lbl_out = QtWidgets.QLabel("—")
        tl.addWidget(self.bar_out)
        tl.addWidget(self.lbl_out)
        torque.setToolTip("Ángulo de campo que pide el lazo, contra el máximo de 81° eléctricos.\n"
                          "Cerca del 100 % el motor se queda sin margen (fricción, velocidad alta).")
        self.r_enc = Readout("Encoder", "estado", "Bits de estado del MT6835")
        grid.addWidget(torque, 1, 0, 1, 2)
        grid.addWidget(self.r_enc, 1, 2)
        self.r_stats = Readout("Ventana del gráfico", "µm", "Error de seguimiento en la ventana visible")
        grid.addWidget(self.r_stats, 1, 3)
        for c in range(4):
            grid.setColumnStretch(c, 1)
        lay.addLayout(grid)

        # carril
        rail_box = QtWidgets.QFrame()
        rail_box.setObjectName("readout")
        rl = QtWidgets.QVBoxLayout(rail_box)
        rl.setContentsMargins(6, 4, 6, 2)
        rl.addWidget(QtWidgets.QLabel(f"Carril <span style='color:{MUTED}'>— azul: carro · naranja: destino · "
                                      f"clic para elegir destino</span>", objectName="readoutTitle"))
        self.rail = RailWidget()
        self.rail.picked.connect(self._rail_picked)
        rl.addWidget(self.rail)
        lay.addWidget(rail_box)

        # controles de los gráficos
        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Ventana:"))
        self.window_combo = QtWidgets.QComboBox()
        for s in (5, 10, 30, 60, 120):
            self.window_combo.addItem(f"{s} s", s)
        self.window_combo.setCurrentIndex(1)
        bar.addWidget(self.window_combo)
        self.chk_pause = QtWidgets.QCheckBox("Pausar gráficos")
        self.chk_pause.toggled.connect(lambda on: setattr(self, "paused", on))
        bar.addWidget(self.chk_pause)
        btn_clear = QtWidgets.QPushButton("Borrar", toolTip="Borrar los gráficos")
        btn_clear.clicked.connect(self.hist.clear)
        bar.addWidget(btn_clear)
        bar.addStretch(1)
        self.btn_rec = QtWidgets.QPushButton("●  Grabar CSV", checkable=True)
        self.btn_rec.setToolTip("Guarda toda la telemetría (1 kHz) y los comandos en mediciones/AAAA-MM-DD/")
        self.btn_rec.toggled.connect(self.toggle_record)
        bar.addWidget(self.btn_rec)
        lay.addLayout(bar)

        # gráficos
        pg.setConfigOptions(antialias=False, background=SURFACE, foreground=INK2)
        glw = pg.GraphicsLayoutWidget()
        glw.ci.setSpacing(4)
        self.pos_axis = OffsetAxis("mm", self._pos_title)
        self.p_pos = glw.addPlot(row=0, col=0, axisItems={"left": self.pos_axis})
        self.p_err = glw.addPlot(row=1, col=0)
        self.p_out = glw.addPlot(row=2, col=0)
        tick_font = QtGui.QFont()
        tick_font.setPointSize(8)
        for p, title, unit in ((self.p_pos, "Posición", "mm"), (self.p_err, "Error de seguimiento", "µm"),
                               (self.p_out, "Salida del lazo", "° el.")):
            self._set_title(p, title)
            p.setLabel("left", unit)
            p.showGrid(x=True, y=True, alpha=0.25)
            p.setDownsampling(auto=True, mode="peak")
            p.setClipToView(True)
            p.setMouseEnabled(x=False, y=True)
            p.getAxis("left").setWidth(56)
            for ax in ("left", "bottom"):
                p.getAxis(ax).setTickFont(tick_font)
                p.getAxis(ax).enableAutoSIPrefix(False)   # que no aparezca "µm (x0.001)"
            if p is not self.p_out:
                p.getAxis("bottom").setStyle(showValues=False)
            if p is not self.p_pos:
                p.setXLink(self.p_pos)
        self.p_pos.setLimits(minYRange=POS_PLOT_MIN_SPAN_MM)
        self.p_out.setLabel("bottom", "tiempo", "s")
        self.p_pos.addLegend(offset=(8, 4), labelTextColor=INK2)
        self.c_target = self.p_pos.plot(pen=pg.mkPen(MUTED, width=1.5, style=Qt.DashLine), name="objetivo")
        self.c_pos = self.p_pos.plot(pen=pg.mkPen(BLUE, width=1.5), name="medida")
        self.p_err.addLine(y=0, pen=pg.mkPen(GRID, width=1))
        self.c_err = self.p_err.plot(pen=pg.mkPen(BLUE, width=1))
        for y in (OUT_LIMIT_DEG, -OUT_LIMIT_DEG):
            self.p_out.addLine(y=y, pen=pg.mkPen(RED, width=1, style=Qt.DashLine))
        self.c_out = self.p_out.plot(pen=pg.mkPen(BLUE, width=1))
        glw.scene().sigMouseClicked.connect(self._plot_clicked)
        lay.addWidget(glw, 1)
        return panel

    # ----------------------------------------------------------------------------------
    # Conexión
    # ----------------------------------------------------------------------------------
    def refresh_ports(self, prefer=None):
        current = prefer or self.port_combo.currentText()
        self.port_combo.clear()
        ports = find_ports()
        if prefer and prefer not in ports:
            ports.insert(0, prefer)
        self.port_combo.addItems(ports)
        if current:
            self.port_combo.setCurrentText(current)

    def toggle_connection(self):
        if self.link.is_open:
            self.disconnect_port()
        else:
            self.connect_port()

    def connect_port(self):
        port = self.port_combo.currentText().strip()
        if not port:
            self.message("No hay puerto: conectar la placa por USB y apretar ↻", RED)
            return
        try:
            self.link.open(port)
        except (serial.SerialException, OSError) as e:
            busy = "ocupado" if "lock" in str(e).lower() or "busy" in str(e).lower() else ""
            hint = " (¿hay un monitor serie u otro script abierto?)" if busy else ""
            self.message(f"No se pudo abrir {port}: {e}{hint}", RED)
            return
        self.hist.t_last_us = None
        self.hist.seq_last = None
        self._spin_init = False
        self.user_relative = False
        self.last_tel_time = 0.0
        self.message(f"Conectado a {port}")
        self.log_line(f"— conectado a {port} —", "info")
        self.link.send("M120 S0", silent=True)
        self.link.send("G90", silent=True)
        self.link.send("M203", silent=True, on_done=self._got_max_speed)
        self.link.send("M119", silent=True, on_done=self._got_status)
        self.link.send(f"M120 S1 P{TELEMETRY_DECIMATION}", silent=True, on_done=self._telemetry_on)
        self.update_controls()

    def disconnect_port(self):
        if self.btn_rec.isChecked():
            self.btn_rec.setChecked(False)
        self.link.close()
        self.homed = self.calibrated = self.loop_on = self.fault = None
        self.busy = None
        self.log_line("— desconectado —", "info")
        self.message("Desconectado")
        self.update_controls()

    def on_lost(self, err):
        self.link.close(stop_telemetry=False)
        self.homed = self.calibrated = self.loop_on = self.fault = None
        self.busy = None
        if self.btn_rec.isChecked():
            self.btn_rec.setChecked(False)
        self.log_line(f"— se perdió la conexión: {err} —", "error")
        self.message("Se perdió la conexión con la placa (¿se desenchufó o se reinició?). "
                     "Reconectar con ↻ y Conectar.", RED)
        self.update_controls()

    def _telemetry_on(self, cmd):
        if not cmd.ok:
            self.message("La placa no aceptó M120: sin telemetría", RED)

    def _got_max_speed(self, cmd):
        for line in cmd.lines:
            if line.startswith("max_velocidad="):
                try:
                    self.fw_max_speed = float(line.split("=")[1].split()[0])
                except ValueError:
                    pass

    def _got_status(self, cmd):
        if cmd.ok is not True:
            return
        st = parse_status(cmd.lines)
        if "homed" in st:
            self.homed = st["homed"] == "1"
            self.calibrated = st.get("calibrated") == "1"
        if "fault" in st and not self.telemetry_alive():
            self.fault = st["fault"] == "1"
            self.loop_on = st.get("motor_update") == "1"
        if "target_mm" in st and self.moves_pending == 0:
            try:
                self.dest = float(st["target_mm"])
            except ValueError:
                pass
        if self.dest is not None and not self._spin_init:
            self.spin_dest.setValue(self.dest)
            self._spin_init = True
        self.update_controls()

    def poll_status(self):
        if self.link.is_open and self.busy is None and self.link.pending() == 0:
            self.link.send("M119", silent=True, on_done=self._got_status)

    def telemetry_alive(self):
        return time.monotonic() - self.last_tel_time < 1.0

    # ----------------------------------------------------------------------------------
    # Comandos
    # ----------------------------------------------------------------------------------
    def can_move(self):
        return (self.link.is_open and self.busy is None and bool(self.homed) and bool(self.calibrated)
                and bool(self.loop_on) and not self.fault)

    def move_to(self, target_mm, speed=None, quiet=False):
        if not self.can_move():
            self.message(self.why_cannot_move(), AMBER)
            return
        v = speed if speed is not None else self.spin_speed.value()
        target_mm = clamp(target_mm, AXIS_MIN_MM, AXIS_MAX_MM)
        if v > self.fw_max_speed + 1e-6:
            self.link.send(f"M203 X{v:g}", silent=True, on_done=self._got_max_speed)
            self.fw_max_speed = v
        self.link.send("G90", silent=True)
        self.moves_pending += 1
        self.link.send(f"G1 X{target_mm:.{POS_DECIMALS}f} F{v:g}", silent=quiet, on_done=self._move_done)
        if self.user_relative:
            self.link.send("G91", silent=True)
        self.dest = target_mm

    def _move_done(self, cmd):
        self.moves_pending = max(0, self.moves_pending - 1)
        if cmd.ok is False:
            self.message(cmd.lines[-1] if cmd.lines else "error", RED)
            self.link.send("M119", silent=True, on_done=self._got_status)

    def why_cannot_move(self):
        if not self.link.is_open:
            return "No hay conexión con la placa"
        if self.busy:
            return f"Esperando: {self.busy}"
        if not self.calibrated or not self.homed:
            return "Primero «Calibrar e iniciar» (M56)"
        if self.fault:
            return "Hubo un corte por falla: revisar el carro y «Habilitar lazo»"
        if not self.loop_on:
            return "El lazo está apagado: «Habilitar lazo» (M17)"
        return ""

    def go(self):
        self.spin_dest.interpretText()
        self.move_to(self.spin_dest.value())

    def step_mm(self):
        return 10.0 ** self.step_exp

    def step_scale(self, k):
        self.step_exp = clamp(self.step_exp + k, STEP_EXP_MIN, STEP_EXP_MAX)
        self.lbl_step.setText(fmt_len(self.step_exp))
        self.btn_step_down.setEnabled(self.step_exp > STEP_EXP_MIN)
        self.btn_step_up.setEnabled(self.step_exp < STEP_EXP_MAX)

    def jog(self, sign):
        if not self.can_move():
            self.message(self.why_cannot_move(), AMBER)
            return
        base = self.dest if self.dest is not None else (self.last[2] if self.last is not None else 0.0)
        target = clamp(round(base + sign * self.step_mm(), POS_DECIMALS), AXIS_MIN_MM, AXIS_MAX_MM)
        if target == base:
            self.message("Fin del recorrido", AMBER)
            return
        self.move_to(target)

    def hold_start(self, sign):
        self.move_to(AXIS_MAX_MM if sign > 0 else AXIS_MIN_MM)

    def hold_stop(self):
        if self.can_move():
            self.stop_motion()

    def stop_motion(self):
        """Frena donde está el objetivo ahora. El perfil del firmware arranca cada G1 desde
        velocidad cero, así que es una frenada en seco (unos µm de error, ver la guía)."""
        if not self.link.is_open or self.busy is not None:
            return
        self.tick_plots()              # la telemetría más fresca: la del último cuadro puede tener 40 ms
        if self.last is None or not self.telemetry_alive():
            self.message("Sin telemetría: no se sabe dónde frenar. Usar «Apagar motor» si hace falta.", AMBER)
            return
        stop_at = clamp(self.last[1] + self.cmd_vel * STOP_LEAD_S, AXIS_MIN_MM, AXIS_MAX_MM)
        self.moves_pending += 1
        self.link.send_urgent(["G90"], silent=True)
        self.link.send(f"G1 X{stop_at:.{POS_DECIMALS}f} F{self.spin_speed.value():g}", on_done=self._move_done)
        if self.user_relative:
            self.link.send("G91", silent=True)
        self.dest = stop_at
        self.message("Detenido")

    def motor_off(self):
        if not self.link.is_open:
            return
        self.link.send_urgent(["M18"])
        self.link.send("M119", silent=True, on_done=self._got_status)
        self.loop_on = False
        self.message("Motor apagado. Para volver a mover: «Habilitar lazo».", AMBER)

    def enable_loop(self):
        if not self.link.is_open:
            return
        self.link.send("M17", on_done=self._enable_done)

    def _enable_done(self, cmd):
        if cmd.ok:
            self.loop_on = True
            self.fault = False
            self.dest = None
            self.message("Lazo habilitado: el carro queda quieto donde está", GREEN)
        else:
            self.message(cmd.lines[-1] if cmd.lines else "M17 sin respuesta", RED)
        self.link.send("M119", silent=True, on_done=self._got_status)

    def calibrate(self):
        if not self.link.is_open or self.busy is not None:
            return
        ans = QtWidgets.QMessageBox.question(
            self, "Calibrar e iniciar",
            "Esto tarda unos 3 minutos y mueve el carro:\n\n"
            "  1. lo lleva contra el tope de 0 mm (homing),\n"
            "  2. calibra el encoder,\n"
            "  3. va y vuelve dos veces entre 0 y 50 mm para aprender el anti-cogging.\n\n"
            "Asegurate de que el recorrido esté libre. Mientras tanto la placa no atiende "
            "otros comandos (ni «Apagar motor»).\n\n¿Empezar?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if ans == QtWidgets.QMessageBox.Yes:
            self.send_long("M56")

    def send_long(self, text):
        cmd = Command(text)
        self.busy = LONG_LABELS.get(cmd.code, text)
        self.busy_since = time.monotonic()
        self.busy_detail = ""
        self.update_controls()
        self.link.send(text, on_done=self._long_done)

    def _long_done(self, cmd):
        self.busy = None
        if cmd.ok:
            extra = [l for l in cmd.lines if not l.startswith("ok")]
            cut = [l for l in extra if "cortado" in l or "no pudo" in l or "no se asienta" in l]
            if cut:
                # calibró, pero el anti-cogging no terminó: el carro puede no estar en 0
                self.message(f"{cmd.code} terminado con avisos: {cut[-1]}", AMBER)
                QtWidgets.QMessageBox.warning(self, cmd.code, "La calibración terminó, pero el anti-cogging no:\n\n"
                                              + "\n".join(extra[-6:]))
            else:
                self.message(f"{cmd.code} terminado" + (f": {extra[-1]}" if extra else ""), GREEN)
            if cmd.code in ("M56", "G28"):
                self.link.send("M119", silent=True, on_done=self._after_long_status)
        else:
            why = cmd.lines[-1] if cmd.lines else "sin respuesta"
            self.message(f"{cmd.code} falló: {why}", RED)
            QtWidgets.QMessageBox.warning(self, cmd.code, f"{cmd.code} falló:\n\n" + "\n".join(cmd.lines[-6:]))
        self.link.send("M119", silent=True, on_done=self._got_status)
        self.update_controls()

    def _after_long_status(self, cmd):
        self._got_status(cmd)
        st = parse_status(cmd.lines)
        if st.get("motor_update") != "1" and self.calibrated and self.homed:
            self.enable_loop()
        elif st.get("motor_update") == "1":
            self.loop_on, self.fault = True, False

    def send_typed(self):
        text = self.gcode.text().strip()
        if not text:
            return
        self.gcode.push(text)
        self.gcode.clear()
        self.send_console(text)

    def send_console(self, text):
        if not self.link.is_open:
            self.message("No hay conexión", RED)
            return
        code = text.split()[0].upper()
        if code in ("G90", "G91"):
            self.user_relative = code == "G91"
        if code in LONG_LABELS:
            self.send_long(text)
        elif code in ("G0", "G1"):
            self.moves_pending += 1
            self.link.send(text, on_done=self._move_done)
        else:
            self.link.send(text, on_done=lambda c: self.link.send("M119", silent=True, on_done=self._got_status))

    # ----------------------------------------------------------------------------------
    # Respuestas y consola
    # ----------------------------------------------------------------------------------
    def on_done(self, cmd):
        if cmd.on_done is not None:
            cmd.on_done(cmd)

    def on_console(self, text, kind):
        if kind == "log" and self.busy:
            self.busy_detail = text[2:].strip()
        if kind == "cmd" and self.rec_file:
            self.rec_file.write(f"# {self.hist.t:.4f} > {text}\n")
        if kind == "cmd":
            self.log_line("> " + text, "cmd")
        elif kind == "reply":
            self.log_line(text, "ok" if text.startswith("ok") else "error" if text.startswith("error") else "reply")
        else:
            self.log_line(text, "error" if text.startswith("E)") else "warn" if text.startswith("W)") else "log")

    def log_line(self, text, kind):
        color = {"cmd": BLUE, "ok": MUTED, "error": RED, "warn": AMBER, "log": INK2, "info": MUTED,
                 "reply": INK}[kind]
        weight = "600" if kind == "cmd" else "400"
        esc = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        self.console_view.appendHtml(f"<span style='color:{color};font-weight:{weight}'>{esc}</span>")

    def message(self, text, color=INK):
        self.status_msg.setText(text)
        self.status_msg.setStyleSheet(f"color:{color}")

    # ----------------------------------------------------------------------------------
    # Telemetría y pantalla
    # ----------------------------------------------------------------------------------
    def tick_plots(self):
        rows = self.link.take_rows()
        if rows:
            new = self.hist.add(rows)
            self.last_tel_time = time.monotonic()
            self._rate_count += len(rows)
            self.last = new[:, -1].copy()
            flags = int(self.last[History.FLAGS])
            if not self.busy:
                self.loop_on = bool(flags & 1)
                self.fault = bool(flags & 2)
            recent = self.hist.last(0.02)
            if recent.shape[1] > 2 and recent[0, -1] > recent[0, 0]:
                self.cmd_vel = (recent[1, -1] - recent[1, 0]) / (recent[0, -1] - recent[0, 0])
            if self.rec_file:
                self._record(new)
        if self.paused or self.hist.n == 0:
            return
        win = self.window_combo.currentData()
        d = self.hist.last(win)
        if d.shape[1] < 2:
            return
        t = d[History.T] - self.hist.t
        off = self._pos_offset(d)
        self.c_target.setData(t, d[History.TARGET] - off)
        self.c_pos.setData(t, d[History.POS] - off)
        err = (d[History.TARGET] - d[History.POS]) * 1000.0
        self.c_err.setData(t, err)
        self.c_out.setData(t, d[History.OUT])
        self.p_pos.setXRange(-win, 0, padding=0.01)
        for p, y in ((self.p_pos, d[History.POS, -1] - off), (self.p_err, err[-1]), (self.p_out, d[History.OUT, -1])):
            self._follow(p, y)
        self._win_err = err
        self._win_out = d[History.OUT]

    @staticmethod
    def _follow(p, y):
        """Un toque de rueda o un arrastre apaga el autoescalado de pyqtgraph para siempre: con
        zoom a mano se vuelve a autoescalar en cuanto el último punto se sale de la vista."""
        vb = p.getViewBox()
        if not vb.autoRangeEnabled()[1] and np.isfinite(y):
            y0, y1 = vb.viewRange()[1]
            if not y0 <= y <= y1:
                vb.enableAutoRange(y=True)

    def _plot_clicked(self, ev):
        if ev.double():                # doble clic: volver al autoescalado
            for p in (self.p_pos, self.p_err, self.p_out):
                if p.sceneBoundingRect().contains(ev.scenePos()):
                    p.getViewBox().enableAutoRange(y=True)

    @staticmethod
    def _set_title(p, title, note=""):
        if note:
            note = f"&nbsp;&nbsp;<span style='color:{MUTED}'>{note}</span>"
        p.setTitle(f"<span style='color:{INK};font-weight:600'>{title}</span>{note}", justify="left", size="9pt")

    def _pos_title(self, note):
        if note != self._pos_note:
            self._pos_note = note
            self._set_title(self.p_pos, "Posición", note)

    def _pos_offset(self, d):
        """Referencia (redonda) que se resta a la posición para graficarla cerca de cero.

        Se queda quieta mientras sirva, para que los rótulos no bailen: tiene que ser múltiplo
        de la potencia de 10 que cubre lo que se ve (así las marcas caen en números redondos)
        y estar a menos de 5 de esas potencias del centro.
        """
        vb = self.p_pos.getViewBox()
        old = self.pos_axis.offset
        if vb.autoRangeEnabled()[1]:
            lo = min(np.nanmin(d[History.TARGET]), np.nanmin(d[History.POS]))
            hi = max(np.nanmax(d[History.TARGET]), np.nanmax(d[History.POS]))
            if not (np.isfinite(lo) and np.isfinite(hi)):
                return old
        else:
            lo, hi = (v + old for v in vb.viewRange()[1])
        step = 10.0 ** math.ceil(math.log10(max(hi - lo, POS_PLOT_MIN_SPAN_MM)))
        center = (lo + hi) / 2
        if abs(old / step - round(old / step)) < 1e-6 and abs(center - old) <= 5 * step:
            return old
        new = round(center / step) * step
        if not vb.autoRangeEnabled()[1]:      # zoom a mano: que la vista no salte
            y0, y1 = vb.viewRange()[1]
            vb.setYRange(y0 + old - new, y1 + old - new, padding=0)
        self.pos_axis.set_offset(new, step)
        return new

    def tick_ui(self):
        now = time.monotonic()
        if now - self._rate_t0 >= 1.0:
            self.rate = self._rate_count / (now - self._rate_t0)
            self._rate_count = 0
            self._rate_t0 = now
        alive = self.link.is_open and self.telemetry_alive()
        if self.link.is_open:
            lost = 100.0 * self.hist.lost / max(1, self.hist.lost + self.hist.received)
            self.status_tel.setText(f"telemetría {self.rate:4.0f} pts/s · {lost:.0f} % perdidos"
                                    if alive else "sin telemetría")
            self.status_tel.setStyleSheet(f"color:{MUTED if alive else AMBER}")
        else:
            self.status_tel.setText("")

        if alive:
            d = self.hist.last(0.1)
            pos = self.last[History.POS]
            err = float(np.mean(d[History.TARGET] - d[History.POS])) * 1000.0
            vel = float(np.mean(d[History.VEL]))
            out = float(np.mean(d[History.OUT]))
            status = int(self.last[History.STATUS])
            self.r_pos.set(f"{pos:.{POS_DECIMALS}f}")
            self.r_err.set(f"{err:+.3f}", RED if abs(err) > 100 else AMBER if abs(err) > 10 else INK)
            self.r_vel.set(f"{vel:+.3f}")
            pct = abs(out) / OUT_LIMIT_DEG * 100
            self.bar_out.setValue(int(min(100, pct)))
            col = RED if pct > 85 else AMBER if pct > 60 else GREEN
            if col != getattr(self, "_bar_col", None):
                self._bar_col = col
                self.bar_out.setStyleSheet(f"QProgressBar::chunk{{background:{col};border-radius:3px}}")
            self.lbl_out.setText(f"{out:+.1f}° de ±{OUT_LIMIT_DEG:.0f}°  ({pct:.0f} %)")
            bits = [name for bit, name in STATUS_BITS.items() if status & bit]
            self.r_enc.set(", ".join(bits) if bits else "OK", RED if bits else GREEN)
            self.rail.set_state(pos, self.dest)
            if getattr(self, "_win_err", None) is not None and len(self._win_err):
                rms = float(np.sqrt(np.mean(self._win_err ** 2)))
                mx = float(np.max(np.abs(self._win_err)))
                self.r_stats.set(f"RMS {rms:.2f} · máx {mx:.1f}")
        else:
            for r in (self.r_pos, self.r_err, self.r_vel, self.r_enc, self.r_stats):
                r.set("—", MUTED)
            self.bar_out.setValue(0)
            self.lbl_out.setText("—")
            self.rail.set_state(None, self.dest)
        self.r_dest.set(f"{self.dest:.{POS_DECIMALS}f}" if self.dest is not None else "—")
        self.update_controls()

    def update_controls(self):
        open_ = self.link.is_open
        self.btn_connect.setText("Desconectar" if open_ else "Conectar")
        self.port_combo.setEnabled(not open_)
        self.chip_conn.setText("Conectado" if open_ else "Desconectado")
        self.chip_conn.set_state(True if open_ else None)
        self.chip_cal.set_state(self.calibrated if open_ else None)
        self.chip_home.set_state(self.homed if open_ else None)
        if not open_ or self.loop_on is None:
            self.chip_loop.setText("Lazo")
            self.chip_loop.set_state(None)
        elif self.busy:
            self.chip_loop.setText("Ocupado")
            self.chip_loop.set_state("warn")
        elif self.fault:
            self.chip_loop.setText("FALLA")
            self.chip_loop.set_state(False)
        else:
            self.chip_loop.setText("Lazo activo" if self.loop_on else "Lazo apagado")
            self.chip_loop.set_state(True if self.loop_on else "warn")

        idle = open_ and self.busy is None
        movable = self.can_move()
        self.btn_cal.setEnabled(idle)
        self.btn_enable.setEnabled(idle and bool(self.calibrated) and bool(self.homed))
        for w in [self.btn_go, self.btn_minus, self.btn_plus, self.btn_hold_minus, self.btn_hold_plus,
                  self.btn_stop] + self.preset_btns:
            # no deshabilitar un botón de mantener mientras está apretado: no llegaría el «soltar»
            if not getattr(w, "isDown", lambda: False)():
                w.setEnabled(movable)
        self.btn_off.setEnabled(open_)
        self.gcode.setEnabled(open_)
        if hasattr(self, "enc_tab"):
            self.enc_tab.update_controls()

        # aviso
        if not open_:
            self._banner(None)
        elif self.busy:
            el = time.monotonic() - self.busy_since
            detail = f" — {self.busy_detail}" if self.busy_detail else ""
            self._banner(f"⏳ {self.busy}: {int(el // 60)}:{int(el % 60):02d}{detail}", "busy")
        elif self.fault:
            self._banner("⚠ Corte por falla: el error de seguimiento pasó de 1 mm durante 0,5 s y el firmware apagó "
                         "el motor. Revisar que nada trabe el carro y apretar «Habilitar lazo».", "error")
        elif self.calibrated is False or self.homed is False:
            self._banner("El eje no está calibrado. Apretar «Calibrar e iniciar» (hace falta en cada encendido).",
                         "warn")
        elif self.loop_on is False:
            self._banner("El lazo está apagado: el motor no sostiene el carro. «Habilitar lazo» para mover.", "warn")
        else:
            self._banner(None)

    def _banner(self, text, kind="warn"):
        if text is None:
            self.banner.hide()
            return
        if (text, kind) == getattr(self, "_banner_now", None) and self.banner.isVisible():
            return
        self._banner_now = (text, kind)
        bg = {"error": "#fbe3e3", "warn": "#fdf1dc", "busy": "#e3eefb"}[kind]
        fg = {"error": RED, "warn": "#8a5a00", "busy": "#184f95"}[kind]
        self.banner.setStyleSheet(f"background:{bg}; color:{fg}; padding:8px 12px; border-radius:6px;"
                                  f"font-weight:600")
        self.banner.setText(text)
        self.banner.show()

    def _rail_picked(self, mm):
        self.spin_dest.setValue(mm)
        self.message(f"Destino {mm:g} mm: apretar «Ir» para mover")

    def _speed_changed(self, v):
        if v > SPEED_SAFE + 1e-9:
            self.lbl_speed_warn.setText(f"Más de {SPEED_SAFE:g} mm/s: el anti-cogging se apaga y el error "
                                        f"crece; a 4 mm/s puede faltar torque.")
        else:
            self.lbl_speed_warn.setText("")

    # ----------------------------------------------------------------------------------
    # Grabación
    # ----------------------------------------------------------------------------------
    def toggle_record(self, on):
        if on:
            if not self.link.is_open:
                self.btn_rec.setChecked(False)
                return
            now = datetime.datetime.now()
            folder = os.path.join(REPO, "mediciones", now.strftime("%Y-%m-%d"))
            os.makedirs(folder, exist_ok=True)
            self.rec_path = os.path.join(folder, now.strftime("interfaz_%Y%m%d_%H%M%S.csv"))
            self.rec_file = open(self.rec_path, "w")
            # encabezado primero: lo leen genfromtxt(names=True) y pandas.read_csv(comment="#")
            self.rec_file.write("t_s,objetivo_mm,posicion_mm,error_um,velocidad_mm_s,salida_deg,estado_encoder,flags\n"
                                f"# μMachining, grabación de tools/interfaz.py, {now:%Y-%m-%d %H:%M:%S}\n"
                                f"# telemetría a {2000 // TELEMETRY_DECIMATION} Hz; las líneas '# t > G1 ...' "
                                f"son los comandos mandados (t en s, mismo reloj)\n")
            self.btn_rec.setText("■  Detener grabación")
            self.status_rec.setText(f"● grabando {os.path.relpath(self.rec_path, REPO)}")
            self.status_rec.setStyleSheet(f"color:{RED}")
        elif self.rec_file:
            self.rec_file.close()
            self.rec_file = None
            self.btn_rec.setText("●  Grabar CSV")
            self.status_rec.setText("")
            self.message(f"Grabación guardada en {os.path.relpath(self.rec_path, REPO)}", GREEN)
            self.log_line(f"— grabación guardada: {self.rec_path} —", "info")

    def _record(self, new):
        err = (new[History.TARGET] - new[History.POS]) * 1000.0
        cols = np.vstack([new[History.T], new[History.TARGET], new[History.POS], err, new[History.VEL],
                          new[History.OUT], new[History.STATUS], new[History.FLAGS]]).T
        np.savetxt(self.rec_file, cols, fmt=["%.4f", "%.7f", "%.7f", "%.4f", "%.4f", "%.2f", "%d", "%d"],
                   delimiter=",")

    def closeEvent(self, ev):
        if self.btn_rec.isChecked():
            self.btn_rec.setChecked(False)
        self.link.close()
        super().closeEvent(ev)


STYLE = f"""
QWidget {{ font-size: 10pt; color: {INK}; }}
QMainWindow, QWidget#central {{ background: {SURFACE}; }}
QGroupBox {{ font-weight: 600; border: 1px solid {GRID}; border-radius: 8px; margin-top: 12px;
             padding: 10px 8px 8px 8px; background: white; }}
QGroupBox::title {{ subcontrol-origin: margin; left: 10px; padding: 0 4px; color: {INK2}; }}
QFrame#readout {{ background: white; border: 1px solid {GRID}; border-radius: 8px; }}
QLabel#readoutTitle {{ color: {INK2}; font-size: 9pt; }}
QLabel#readoutValue {{ font-family: 'DejaVu Sans Mono', monospace; font-size: 19pt; font-weight: 600; }}
QPushButton {{ padding: 6px 10px; border: 1px solid #c3c2b7; border-radius: 6px; background: #f6f5f2; }}
QPushButton:hover {{ background: #eeede8; }}
QPushButton:pressed {{ background: #e1e0d9; }}
QPushButton:disabled {{ color: #b5b3ac; background: #f3f2ef; border-color: #e1e0d9; }}
QPushButton#primary {{ background: {BLUE}; color: white; border-color: #256abf; font-weight: 600; min-width: 70px; }}
QPushButton#primary:disabled {{ background: #a9c6ea; border-color: #a9c6ea; }}
QPushButton#big {{ padding: 12px; font-size: 12pt; font-weight: 600; }}
QPushButton#stop {{ padding: 10px; font-weight: 600; background: #fdf1dc; border-color: {AMBER}; color: #8a5a00; }}
QPushButton#stop:disabled {{ background: #f3f2ef; border-color: #e1e0d9; color: #b5b3ac; }}
QPushButton#danger {{ background: {RED}; color: white; border-color: #a82e2e; font-weight: 700; padding: 8px 16px; }}
QPushButton#danger:disabled {{ background: #e9b3b3; border-color: #e9b3b3; }}
QLabel#stepValue {{ font-family: 'DejaVu Sans Mono', monospace; font-size: 12pt; font-weight: 600; }}
QPushButton#seg {{ padding: 4px 6px; }}
QPushButton#seg:checked {{ background: {BLUE}; color: white; border-color: #256abf; }}
QPushButton:checked {{ background: #fbe3e3; border-color: {RED}; color: {RED}; }}
QPlainTextEdit#console {{ font-family: 'DejaVu Sans Mono', monospace; font-size: 9pt; background: #fafaf8;
                          border: 1px solid {GRID}; border-radius: 6px; }}
QProgressBar {{ border: 1px solid {GRID}; border-radius: 4px; background: #f3f2ef; }}
QLabel#resValue {{ font-family: 'DejaVu Sans Mono', monospace; font-size: 10pt; font-weight: 600; }}
QTabWidget#mainTabs::pane {{ border: none; }}
QTabBar::tab {{ padding: 6px 16px; border: 1px solid {GRID}; border-bottom: none; border-top-left-radius: 6px;
                border-top-right-radius: 6px; background: #f3f2ef; color: {INK2}; margin-right: 2px; }}
QTabBar::tab:selected {{ background: white; color: {INK}; border-color: #c3c2b7; }}
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="puerto serie (por defecto, el primero que parezca una Pico)")
    args = ap.parse_args()

    QtCore.QCoreApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName("μMachining")
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    win = MainWindow(args.port)
    win.resize(1500, 920)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
