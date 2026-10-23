#!/usr/bin/env python3
"""Interfaz gráfica del eje: control remoto y telemetría en vivo.

    python3 tools/interfaz.py                    # busca la placa y se conecta sola
    python3 tools/interfaz.py --port /dev/ttyACM1

Guía de uso: docs/GUIA_INTERFAZ.md. Cerrar antes cualquier otro programa que tenga el
puerto abierto (monitor serie de PlatformIO, otros scripts de tools/).

No cambia nada del firmware: manda los mismos comandos G-code que se escriben a mano y
prende la telemetría binaria (M120). Protocolo (ver src/command_processor.cpp y
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

# --- montaje y firmware (ver src/hw_config.h) ---
BAUD = 921600
PITCH_MM = 5.0                 # husillo: mm por vuelta de motor
AXIS_MIN_MM, AXIS_MAX_MM = 0.0, 255.0
POS_DECIMALS = 5              # 10 nm. El piso real es el float32 del firmware (posición en rad):
                              # ~3 nm a 30 mm, ~12 nm a 150 mm, ~24 nm de 204 a 255 mm
POS_PLOT_MIN_SPAN_MM = 0.01    # con el eje quieto el autoescalado haría zoom al ruido de nm y los
                              # rótulos (149.99996) no entran en el eje; el detalle fino va en el error
SPEED_DEFAULT = 2.0            # mm/s: la velocidad para la que se aprende el anti-cogging
SPEED_SAFE = 2.0               # por encima el anti-cogging se desvanece (no está a 3 mm/s)
SPEED_MAX = 5.0                # MAX_VELOCITY_LIMIT_MM_S (a 4 mm/s ya falta torque)
OUT_LIMIT_DEG = 81.0           # límite de la salida del lazo de velocidad (0.45 pi)
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
LONG_LABELS = {"M56": "Calibrando (M56)", "G28": "Homing (G28)", "M57": "Barrido de conmutación (M57)"}

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
    """Atajos: ←/→ un paso, Espacio detener, Esc apagar el motor. Las flechas y el espacio no
    se toman mientras se escribe en un campo de texto o número."""

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
        if key in (Qt.Key_Left, Qt.Key_Right, Qt.Key_Space):
            if not ev.isAutoRepeat():
                if key == Qt.Key_Space:
                    self.win.stop_motion()
                else:
                    self.win.jog(+1 if key == Qt.Key_Right else -1)
            return True
        return False


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
        root.addWidget(split, 1)

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
        self.step_group = QtWidgets.QButtonGroup(self)
        for i, (label, mm) in enumerate((("1 µm", 0.001), ("10 µm", 0.01), ("0,1 mm", 0.1),
                                         ("1 mm", 1.0), ("10 mm", 10.0))):
            btn = QtWidgets.QPushButton(label, checkable=True)
            btn.setObjectName("seg")
            btn.setProperty("step_mm", mm)
            btn.setChecked(mm == 1.0)
            self.step_group.addButton(btn, i)
            steps.addWidget(btn)
        b.addLayout(steps)
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
        self.p_pos = glw.addPlot(row=0, col=0)
        self.p_err = glw.addPlot(row=1, col=0)
        self.p_out = glw.addPlot(row=2, col=0)
        tick_font = QtGui.QFont()
        tick_font.setPointSize(8)
        for p, title, unit in ((self.p_pos, "Posición", "mm"), (self.p_err, "Error de seguimiento", "µm"),
                               (self.p_out, "Salida del lazo", "° el.")):
            p.setTitle(f"<span style='color:{INK};font-weight:600'>{title}</span>", justify="left", size="9pt")
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
        btn = self.step_group.checkedButton()
        return btn.property("step_mm") if btn else 1.0

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
        self.c_target.setData(t, d[History.TARGET])
        self.c_pos.setData(t, d[History.POS])
        err = (d[History.TARGET] - d[History.POS]) * 1000.0
        self.c_err.setData(t, err)
        self.c_out.setData(t, d[History.OUT])
        self.p_pos.setXRange(-win, 0, padding=0.01)
        self._win_err = err
        self._win_out = d[History.OUT]

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
QPushButton#seg {{ padding: 4px 6px; }}
QPushButton#seg:checked {{ background: {BLUE}; color: white; border-color: #256abf; }}
QPushButton:checked {{ background: #fbe3e3; border-color: {RED}; color: {RED}; }}
QPlainTextEdit#console {{ font-family: 'DejaVu Sans Mono', monospace; font-size: 9pt; background: #fafaf8;
                          border: 1px solid {GRID}; border-radius: 6px; }}
QProgressBar {{ border: 1px solid {GRID}; border-radius: 4px; background: #f3f2ef; }}
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
