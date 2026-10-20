#!/usr/bin/env python3
"""Prueba de idas y vueltas en lazo cerrado con captura de telemetria (firmware real).

Habilita el lazo (M17), prende la telemetria binaria (M120 S1 P<n>) y recorre el carril
0 -> X_fin -> 0 las veces pedidas. Guarda todo lo que manda la placa en un .npz y genera
los graficos en mediciones/AAAA-MM-DD/:

  *_resumen.png    todo el carril: error de seguimiento, salida del lazo (margen de torque),
                   velocidad y coherencia del encoder, por tramo (ida/vuelta de cada ciclo)
  *_tiempo.png     la prueba completa en funcion del tiempo
  *_mapa.png       mapa por vueltas de motor de la salida del lazo (media y pico): una fila
                   por vuelta (5 mm), columnas = fase dentro de la vuelta. Muestra de un vistazo si algo se repite en cada vuelta
                   (acople, tabla) o si es de un lugar del carril (husillo, guia)
  *_ventanas.png   el carril en ventanas de --window-mm (por defecto 10 mm = 2 vueltas de
                   motor = 60 periodos del encoder), con los datos crudos de los cuatro tramos
  *_plegado.png    error, salida y velocidad plegados sobre una vuelta de motor y sobre un
                   periodo del encoder (ondulacion por imanes / por tabla)

Uso:
    python3 tools/closed_loop_test.py                       # 2 ciclos 0 -> 255 -> 0
    python3 tools/closed_loop_test.py --cycles 1 --end 100
    python3 tools/closed_loop_test.py --from-file mediciones/AAAA-MM-DD/lazo_cerrado_X.npz

Requiere el eje homeado y calibrado (G28 o M56). Si salta el corte por falla, la prueba se
detiene, guarda M119 y la traza M121 junto al .npz, y grafica lo capturado hasta ahi.
Cerrar antes cualquier otro programa que tenga el puerto serie abierto.

Paquete de telemetria (ver src/shared_data.h): 0xAA 0x55 + 28 bytes little endian
    uint32 t_us, float target_mm, float pos_mm, float vel [rad/s], float out [rad],
    int32 chip_raw, uint8 status, uint8 flags, uint16 seq
"""
import argparse
import datetime
import json
import os
import struct
import sys
import time
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

# --- parametros del montaje (ver src/hw_config.h) ---
PITCH_MM = 5.0                    # husillo: mm por vuelta de motor
ENC_PERIODS_PER_REV = 30          # rueda de 60 imanes
ENC_PERIOD_MM = PITCH_MM / ENC_PERIODS_PER_REV
CPR = 1 << 21                     # MT6835
OUT_LIMIT_DEG = np.degrees(0.45 * np.pi)   # limite de la salida del lazo de velocidad (81°)
FAULT_ERR_MM = 1.0                # corte por falla del firmware: >1 mm durante 0.5 s
TICK_HZ = 2000                    # ISR del perfil

PKT = struct.Struct("<IffffiBBH")
HEADER = b"\xaa\x55"
FIELDS = ["t_us", "target_mm", "pos_mm", "vel", "out", "chip_raw", "status", "flags", "seq"]

STATUS_BITS = {0x01: "overspeed", 0x02: "campo debil", 0x04: "subtension", 0x08: "error CRC"}

# --- estilo ---
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#898781"
GRID = "#e1e0d9"
AXIS = "#c3c2b7"
CRITICAL = "#d03b3b"
DIR_COLOR = {+1: "#2a78d6", -1: "#eb6834"}          # ida azul, vuelta naranja
CYCLE_STYLE = {0: dict(ls=(0, (4, 2)), alpha=0.75), 1: dict(ls="-", alpha=1.0)}
SEQ_BLUE = LinearSegmentedColormap.from_list(
    "azul", ["#f0efec", "#cde2fb", "#86b6ef", "#3987e5", "#256abf", "#184f95", "#0d366b"])

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "font.size": 9, "axes.titlesize": 10, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "legend.frameon": False, "lines.linewidth": 1.2,
})


# ======================================================================================
# Captura
# ======================================================================================

class Link:
    """Puerto serie con texto (respuestas G-code) y paquetes binarios mezclados.

    El texto es ASCII (< 0x80), asi que 0xAA solo puede ser el comienzo de un paquete."""

    def __init__(self, port, baud):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.buf = bytearray()
        self.text = ""
        self.lines = []        # lineas de texto completas pendientes de leer
        self.log = []          # todo el texto recibido, para guardar
        self.rows = []         # paquetes decodificados (tuplas)
        self.leg = []          # tramo de cada paquete
        self.current_leg = -1

    def close(self):
        self.ser.close()

    def poll(self):
        n = self.ser.in_waiting
        data = self.ser.read(n if n else 1)
        if not data:
            return
        self.buf += data
        while True:
            i = self.buf.find(HEADER)
            if i < 0:
                keep = 1 if self.buf.endswith(b"\xaa") else 0
                self._text(self.buf[:len(self.buf) - keep])
                del self.buf[:len(self.buf) - keep]
                return
            if i > 0:
                self._text(self.buf[:i])
                del self.buf[:i]
            if len(self.buf) < 2 + PKT.size:
                return
            self.rows.append(PKT.unpack_from(self.buf, 2))
            self.leg.append(self.current_leg)
            del self.buf[:2 + PKT.size]

    def _text(self, b):
        if not b:
            return
        self.text += b.decode(errors="replace")
        while "\n" in self.text:
            line, self.text = self.text.split("\n", 1)
            line = line.rstrip("\r")
            if line and not line.startswith(">"):
                self.lines.append(line)
                self.log.append(line)

    def command(self, cmd, timeout=10.0, echo=True):
        """Manda un comando y devuelve las lineas de la respuesta (hasta ok/error)."""
        self.lines.clear()
        self.log.append("> " + cmd)
        if echo:
            print("> " + cmd)
        self.ser.write((cmd + "\n").encode())
        out, t0 = [], time.time()
        while time.time() - t0 < timeout:
            self.poll()
            while self.lines:
                line = self.lines.pop(0)
                out.append(line)
                if echo:
                    print("  " + line)
                if line.startswith("ok") or line.startswith("error"):
                    return out
        raise TimeoutError(f"sin respuesta a '{cmd}'")

    def wait_text(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.poll()
        self.lines.clear()


def parse_status(lines):
    st = {}
    for line in lines:
        for tok in line.replace("servo:", "").split():
            if "=" in tok:
                k, v = tok.split("=", 1)
                st[k] = v
    return st


def run_move(link, dest, feed, settle_s, label, abort_err_um=None):
    """G1 hasta dest y espera a que el perfil termine y el carro se asiente.
    Devuelve False si salto el corte por falla, o si |error| pasa abort_err_um (si se da)."""
    link.lines.clear()
    link.command(f"G1 X{dest:g} F{feed:g}", echo=False)
    print(f"[{label}] G1 X{dest:g}")
    t_start = time.time()
    n0 = len(link.rows)
    start = link.rows[-1][2] if link.rows else dest
    timeout = abs(dest - start) / feed + 15.0
    settled_since = None
    last_print = 0.0
    max_out = max_err = 0.0
    while True:
        link.poll()
        if len(link.rows) == n0:
            if time.time() - t_start > 3.0:
                raise RuntimeError("no llega telemetria (M120 S1?)")
            continue
        t_us, tgt, pos, vel, out, raw, status, flags, seq = link.rows[-1]
        max_out = max(max_out, abs(np.degrees(out)))
        max_err = max(max_err, abs(tgt - pos))
        if flags & 2:
            print(f"[{label}] CORTE POR FALLA en pos={pos:.3f} mm (objetivo {tgt:.3f})")
            return False
        if abort_err_um and abs(tgt - pos) * 1000 > abort_err_um:
            print(f"[{label}] ABORTADO: |error| = {1000 * abs(tgt - pos):.0f} um > {abort_err_um:g} um "
                  f"en pos={pos:.3f} mm")
            return False
        now = time.time()
        if now - last_print > 2.0:
            last_print = now
            print(f"   pos={pos:8.3f} mm  err={1000 * (tgt - pos):+7.1f} um  "
                  f"out={np.degrees(out):+6.1f}°  max|out|={max_out:5.1f}°  max|err|={1000 * max_err:6.1f} um")
        if abs(tgt - dest) < 1e-4 and abs(pos - dest) < 0.005:
            settled_since = settled_since or now
            if now - settled_since > settle_s:
                print(f"[{label}] llego: pos={pos:.4f} mm en {now - t_start:.1f} s  "
                      f"max|out|={max_out:.1f}°  max|err|={1000 * max_err:.1f} um")
                return True
        else:
            settled_since = None
        if now - t_start > timeout:
            print(f"[{label}] TIMEOUT: pos={pos:.3f} mm, objetivo {tgt:.3f}")
            return False


def capture(args):
    link = Link(args.port, args.baud)
    meta = {"fecha": datetime.datetime.now().isoformat(timespec="seconds"),
            "end_mm": args.end, "cycles": args.cycles, "feed_mm_s": args.feed,
            "decimation": args.decimation, "legs": [], "ok": True}
    extra = {}
    try:
        time.sleep(0.5)
        link.command("M120 S0")
        link.wait_text(0.3)
        link.buf.clear()
        st = parse_status(link.command("M119"))
        if st.get("homed") != "1" or st.get("calibrated") != "1":
            sys.exit("el eje no esta homeado/calibrado: correr G28 (o M56) primero")
        meta["m119_inicial"] = st
        link.command("G90")
        for c in args.pre:
            meta.setdefault("pre", []).append(c)
            meta.setdefault("pre_reply", []).append(link.command(c))
        link.command("M17")
        link.command(f"M120 S1 P{args.decimation}")
        link.wait_text(0.5)

        # aproximacion al 0 (no cuenta como tramo)
        pos = link.rows[-1][2] if link.rows else 0.0
        if abs(pos) > 0.01:
            if not run_move(link, 0.0, args.feed, args.settle, "aproximacion"):
                meta["ok"] = False

        dests = []
        for c in range(args.cycles):
            dests += [(args.end, +1, c), (0.0, -1, c)]
        for k, (dest, d, c) in enumerate(dests):
            if not meta["ok"]:
                break
            link.current_leg = k
            name = f"{'ida' if d > 0 else 'vuelta'} {c + 1}"
            t0 = time.time()
            ok = run_move(link, dest, args.feed, args.settle, name)
            meta["legs"].append({"leg": k, "name": name, "dir": d, "cycle": c, "dest": dest,
                                 "ok": ok, "dur_s": round(time.time() - t0, 2)})
            link.current_leg = -1
            if not ok:
                meta["ok"] = False
            else:
                link.wait_text(args.dwell)   # reposo en la punta
    except KeyboardInterrupt:
        print("interrumpido")
        meta["ok"] = False
    finally:
        try:
            link.command("M120 S0")
            link.wait_text(0.3)
            link.lines.clear()
            st = link.command("M119")
            meta["m119_final"] = parse_status(st)
            if not meta["ok"]:
                extra["m119"] = st
                extra["m121"] = link.command("M121", timeout=30, echo=False)
        finally:
            link.close()

    data = rows_to_arrays(link.rows, link.leg)
    return data, meta, link.log, extra


def rows_to_arrays(rows, legs):
    a = np.array(rows, dtype=np.float64).reshape(-1, len(FIELDS))
    d = {k: a[:, i] for i, k in enumerate(FIELDS)}
    t = d["t_us"].astype(np.int64)
    dt = np.diff(t) % (1 << 32)                      # reloj de 32 bits
    d["t_s"] = np.concatenate([[0], np.cumsum(dt)]) * 1e-6
    d["leg"] = np.array(legs, dtype=np.int16)
    return d


# ======================================================================================
# Analisis
# ======================================================================================

def derive(d, meta):
    """Magnitudes en unidades de lectura."""
    d["err_um"] = 1000.0 * (d["target_mm"] - d["pos_mm"])
    d["out_deg"] = np.degrees(d["out"])
    d["vel_mm_s"] = d["vel"] / (2 * np.pi) * PITCH_MM
    d["enc_deg"] = d["chip_raw"] / CPR * 360.0
    # Coherencia del encoder: fase del periodo que sale de la posicion menos el angulo crudo
    # del chip. Varia suave y se repite cada vuelta (la tabla corrige la no linealidad);
    # un escalon de 360/n grados indica que el contador de periodos se corrio.
    # El sentido del chip respecto de los mm depende del montaje: se toma el que da una
    # diferencia estable (variacion circular minima entre muestras).
    best = None
    for s in (+1, -1):
        ph = (d["pos_mm"] / ENC_PERIOD_MM) * 360.0 - s * d["enc_deg"]
        jitter = np.mean(np.abs((np.diff(ph) + 180.0) % 360.0 - 180.0)) if len(ph) > 1 else 0
        if best is None or jitter < best[0]:
            best = (jitter, ph)
    ph = best[1]
    ph = (ph - np.degrees(np.angle(np.mean(np.exp(1j * np.radians(ph))))) + 180.0) % 360.0 - 180.0
    d["coh_deg"] = ph
    # velocidad del objetivo, para separar crucero de aceleraciones
    t = d["t_s"]
    vt = np.gradient(d["target_mm"], t) if len(t) > 2 else np.zeros_like(t)
    d["vt_mm_s"] = vt
    d["cruise"] = np.abs(np.abs(vt) - meta["feed_mm_s"]) < 0.02 * meta["feed_mm_s"]


def legs_of(meta):
    return [L for L in meta["legs"]]


def leg_mask(d, k):
    return d["leg"] == k


def summary(d, meta):
    rows = []
    seq = d["seq"].astype(np.int64)
    drops = int(np.sum((np.diff(seq) % 65536) - 1)) if len(seq) > 1 else 0
    for L in legs_of(meta):
        m = leg_mask(d, L["leg"])
        if not m.any():
            continue
        c = m & d["cruise"]
        st = d["status"][m].astype(int)
        rows.append({
            "tramo": L["name"], "ok": L["ok"], "dur_s": float(d["t_s"][m][-1] - d["t_s"][m][0]),
            "desde": float(d["pos_mm"][m][0]), "hasta": float(d["pos_mm"][m][-1]),
            "max_err_um": float(np.max(np.abs(d["err_um"][m]))),
            "rms_err_crucero_um": float(np.sqrt(np.mean(d["err_um"][c] ** 2))) if c.any() else np.nan,
            "out_medio_deg": float(np.mean(d["out_deg"][c])) if c.any() else np.nan,
            "max_out_deg": float(np.max(np.abs(d["out_deg"][m]))),
            "pct_out_70": float(100 * np.mean(np.abs(d["out_deg"][m]) > 70)),
            "n_saturado": int(np.sum(np.abs(d["out_deg"][m]) > OUT_LIMIT_DEG - 0.5)),
            "vel_min_crucero": float(np.min(np.abs(d["vel_mm_s"][c]))) if c.any() else np.nan,
            "status": {name: int(np.sum(st & bit > 0)) for bit, name in STATUS_BITS.items()},
            "coh_pp_deg": float(np.ptp(d["coh_deg"][m])),
        })
    return rows, drops


def print_summary(rows, drops, meta, out=sys.stdout):
    p = lambda s="": print(s, file=out)
    if meta.get("pre"):
        p("comandos previos: " + ", ".join(meta["pre"]))
    p(f"Prueba {meta['fecha']}: {meta['cycles']} ciclos 0 -> {meta['end_mm']:g} mm a "
      f"{meta['feed_mm_s']:g} mm/s, telemetria a {TICK_HZ / meta['decimation']:.0f} Hz")
    p(f"Resultado: {'OK, sin cortes' if meta['ok'] else 'FALLO / INTERRUMPIDA'}   "
      f"paquetes perdidos: {drops}")
    p()
    p(f"{'tramo':9s} {'dur[s]':>7s} {'max|err|':>9s} {'rms err':>8s} {'out medio':>9s} "
      f"{'max|out|':>8s} {'>70°':>6s} {'satur.':>6s} {'vmin':>6s} {'coh pp':>7s}  estado encoder")
    p(f"{'':9s} {'':>7s} {'[um]':>9s} {'[um]':>8s} {'[°el]':>9s} {'[°el]':>8s} {'[%]':>6s} "
      f"{'[mues]':>6s} {'[mm/s]':>6s} {'[°]':>7s}")
    for r in rows:
        flags = ", ".join(f"{k}={v}" for k, v in r["status"].items() if v) or "limpio"
        p(f"{r['tramo']:9s} {r['dur_s']:7.1f} {r['max_err_um']:9.1f} {r['rms_err_crucero_um']:8.1f} "
          f"{r['out_medio_deg']:+9.1f} {r['max_out_deg']:8.1f} {r['pct_out_70']:6.2f} "
          f"{r['n_saturado']:6d} {r['vel_min_crucero']:6.2f} {r['coh_pp_deg']:7.1f}  {flags}")
    p(f"\nmargen de torque: limite de la salida = ±{OUT_LIMIT_DEG:.0f}° electricos; "
      f"corte por falla con |err| > {1000 * FAULT_ERR_MM:.0f} um durante 0,5 s")


# ======================================================================================
# Graficos
# ======================================================================================

def binned(x, y, edges):
    """media, minimo y maximo de y en cada intervalo de x."""
    idx = np.digitize(x, edges) - 1
    n = len(edges) - 1
    ok = (idx >= 0) & (idx < n)
    idx, y = idx[ok], y[ok]
    cnt = np.bincount(idx, minlength=n).astype(float)
    mean = np.bincount(idx, y, minlength=n) / np.where(cnt > 0, cnt, np.nan)
    lo = np.full(n, np.nan)
    hi = np.full(n, np.nan)
    np.fmin.at(lo, idx, y)
    np.fmax.at(hi, idx, y)
    return mean, lo, hi


def style_for(L):
    return dict(color=DIR_COLOR[L["dir"]], **CYCLE_STYLE[min(L["cycle"], 1)])


def leg_legend(ax, meta, loc="upper right", ncol=4):
    from matplotlib.lines import Line2D
    h = [Line2D([], [], lw=1.6, **style_for(L)) for L in legs_of(meta)]
    ax.legend(h, [L["name"] for L in legs_of(meta)], loc=loc, ncol=ncol, fontsize=8,
              handlelength=3)


def limit_lines(ax, label=True):
    for s in (+1, -1):
        ax.axhline(s * OUT_LIMIT_DEG, color=CRITICAL, lw=1, ls=(0, (2, 2)))
    if label:
        ax.text(1.0, OUT_LIMIT_DEG, f" límite ±{OUT_LIMIT_DEG:.0f}°", transform=ax.get_yaxis_transform(),
                va="center", ha="left", color=CRITICAL, fontsize=8, clip_on=False)


def fig_resumen(d, meta, rows, base):
    end = meta["end_mm"]
    edges = np.arange(0, end + ENC_PERIOD_MM, ENC_PERIOD_MM)      # un periodo del encoder
    xc = 0.5 * (edges[1:] + edges[:-1])
    panels = [("err_um", "error de seguimiento [µm]\n(objetivo − posición)"),
              ("out_deg", "salida del lazo [° eléctricos]\n(margen de torque)"),
              ("vel_mm_s", "velocidad [mm/s]"),
              ("coh_deg", "coherencia del encoder [°]\n(fase por posición − ángulo crudo)")]
    fig, axs = plt.subplots(len(panels), 1, figsize=(16, 13), sharex=True,
                            gridspec_kw=dict(hspace=0.12))
    for ax, (key, ylabel) in zip(axs, panels):
        for L in legs_of(meta):
            m = leg_mask(d, L["leg"])
            if not m.any():
                continue
            mean, lo, hi = binned(d["pos_mm"][m], d[key][m], edges)
            st = style_for(L)
            ax.fill_between(xc, lo, hi, color=st["color"], alpha=0.12 if L["cycle"] else 0.08, lw=0)
            ax.plot(xc, mean, lw=1.0, **st)
        ax.set_ylabel(ylabel)
    limit_lines(axs[1])
    axs[1].set_ylim(-OUT_LIMIT_DEG - 12, OUT_LIMIT_DEG + 12)
    axs[2].axhline(meta["feed_mm_s"], color=MUTED, lw=0.8)
    axs[2].axhline(-meta["feed_mm_s"], color=MUTED, lw=0.8)
    # muestras con bits de estado del encoder
    bad = d["status"].astype(int) & 0x0F > 0
    if bad.any():
        axs[3].plot(d["pos_mm"][bad], d["coh_deg"][bad], "x", color=CRITICAL, ms=5,
                    label="bits de estado del encoder")
        axs[3].legend(loc="lower right", fontsize=8)
    for ax in axs:
        for x in np.arange(0, end + 0.1, 50):
            ax.axvline(x, color=AXIS, lw=0.6)
    axs[-1].set_xlabel("posición en el carril [mm]   (cada punto = un período del encoder, "
                       f"{1000 * ENC_PERIOD_MM:.0f} µm: línea = media, banda = mín–máx)")
    axs[-1].set_xlim(0, end)
    leg_legend(axs[0], meta)
    status = "SIN CORTES" if meta["ok"] else "CON FALLA / INTERRUMPIDA"
    worst = max((r["max_out_deg"] for r in rows), default=np.nan)
    worst_err = max((r["max_err_um"] for r in rows), default=np.nan)
    fig.suptitle(f"Lazo cerrado, {meta['cycles']} idas y vueltas 0 → {end:g} mm a {meta['feed_mm_s']:g} mm/s — "
                 f"{status}.  Máx |salida| {worst:.0f}° de {OUT_LIMIT_DEG:.0f}°, máx |error| {worst_err:.0f} µm",
                 x=0.01, ha="left", fontsize=12, fontweight="bold", y=0.995)
    fig.text(0.01, 0.968, f"{meta['fecha']}  ·  ida: salida y error positivos; vuelta: negativos  ·  "
             "línea punteada = ciclo 1, llena = ciclo 2", color=INK2, fontsize=9)
    fig.subplots_adjust(left=0.07, right=0.95, top=0.95, bottom=0.05)
    out = base + "_resumen.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def fig_tiempo(d, meta, base):
    t = d["t_s"]
    fig, axs = plt.subplots(3, 1, figsize=(16, 9), sharex=True, gridspec_kw=dict(hspace=0.1))
    step = max(1, len(t) // 60000)
    s = slice(None, None, step)
    axs[0].plot(t[s], d["target_mm"][s], color=MUTED, lw=2.5, label="objetivo")
    axs[0].plot(t[s], d["pos_mm"][s], color=DIR_COLOR[+1], lw=1.2, label="posición (encoder)")
    axs[0].set_ylabel("posición [mm]")
    axs[0].legend(loc="upper right", ncol=2, fontsize=8)
    axs[1].plot(t[s], d["err_um"][s], color=INK2, lw=0.6)
    axs[1].set_ylabel("error [µm]")
    axs[2].plot(t[s], d["out_deg"][s], color=INK2, lw=0.6)
    limit_lines(axs[2])
    axs[2].set_ylabel("salida [° el.]")
    axs[2].set_xlabel("tiempo [s]")
    for L in legs_of(meta):
        m = leg_mask(d, L["leg"])
        if m.any():
            for ax in axs:
                ax.axvspan(t[m][0], t[m][-1], color=DIR_COLOR[L["dir"]], alpha=0.05, lw=0)
            axs[0].text(0.5 * (t[m][0] + t[m][-1]), 1.0, L["name"], transform=axs[0].get_xaxis_transform(),
                        ha="center", va="bottom", fontsize=9, color=INK2)
    fault = (d["flags"].astype(int) & 2) > 0
    if fault.any():
        i = np.argmax(fault)
        for ax in axs:
            ax.axvline(t[i], color=CRITICAL, lw=1.2)
        axs[0].text(t[i], 0.5, " corte por falla", color=CRITICAL, transform=axs[0].get_xaxis_transform())
    axs[-1].set_xlim(t[0], t[-1])
    fig.suptitle("Prueba completa en el tiempo", x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.06, right=0.95, top=0.92, bottom=0.07)
    out = base + "_tiempo.png"
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return out


def fig_mapa(d, meta, base):
    """Una fila por vuelta de motor, columnas = fase en la vuelta (4 columnas por periodo)."""
    end = meta["end_mm"]
    n_rev = int(np.ceil(end / PITCH_MM))
    n_col = ENC_PERIODS_PER_REV * 4
    legs = legs_of(meta)
    keys = [("out_deg", "|salida| media [° el.]", "media"), ("out_deg", "|salida| pico [° el.]", "pico")]
    fig, axs = plt.subplots(len(keys), len(legs), figsize=(4.0 * len(legs) + 1.5, 13), squeeze=False,
                            sharex=True, sharey=True, gridspec_kw=dict(hspace=0.12, wspace=0.06))
    for r, (key, label, agg) in enumerate(keys):
        grids = []
        for L in legs:
            m = leg_mask(d, L["leg"]) & d["cruise"]
            g = np.full((n_rev, n_col), np.nan)
            if m.any():
                rev = np.floor(d["pos_mm"][m] / PITCH_MM).astype(int)
                col = np.floor((d["pos_mm"][m] % PITCH_MM) / PITCH_MM * n_col).astype(int)
                ok = (rev >= 0) & (rev < n_rev) & (col >= 0) & (col < n_col)
                flat = rev[ok] * n_col + col[ok]
                v = np.abs(d[key][m][ok])
                if agg == "pico":
                    g = np.full(n_rev * n_col, np.nan)
                    np.fmax.at(g, flat, v)
                    g = g.reshape(n_rev, n_col)
                else:
                    cnt = np.bincount(flat, minlength=n_rev * n_col)
                    s = np.bincount(flat, v, minlength=n_rev * n_col)
                    g = (s / np.where(cnt > 0, cnt, np.nan)).reshape(n_rev, n_col)
            grids.append(g)
        top = OUT_LIMIT_DEG
        for c, (L, g) in enumerate(zip(legs, grids)):
            ax = axs[r, c]
            im = ax.imshow(g, aspect="auto", origin="lower", cmap=SEQ_BLUE, vmin=0, vmax=top,
                           extent=[0, PITCH_MM, 0, n_rev * PITCH_MM], interpolation="nearest")
            ax.grid(False)
            if r == 0:
                ax.set_title(L["name"], color=DIR_COLOR[L["dir"]])
            if r == len(keys) - 1:
                ax.set_xlabel("fase en la vuelta de motor [mm]")
            if c == 0:
                ax.set_ylabel("posición en el carril [mm]\n(una fila por vuelta)")
        cb = fig.colorbar(im, ax=axs[r, :].tolist(), fraction=0.02, pad=0.01)
        cb.set_label(label)
        cb.outline.set_visible(False)
        cb.ax.axhline(70, color=CRITICAL, lw=1)
    fig.suptitle("Mapa por vueltas de motor (solo tramos a velocidad de crucero). "
                 "Bandas verticales = se repite en cada vuelta (acople, tabla); "
                 "horizontales = un lugar del carril (husillo, guía)",
                 x=0.01, ha="left", fontsize=11, fontweight="bold")
    out = base + "_mapa.png"
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return out


def fig_ventanas(d, meta, base, win_mm):
    end = meta["end_mm"]
    n_win = int(np.ceil(end / win_mm))
    ncols = 4
    nrows = int(np.ceil(n_win / ncols))
    fig = plt.figure(figsize=(5.0 * ncols, 3.6 * nrows))
    outer = fig.add_gridspec(nrows, ncols, hspace=0.38, wspace=0.18)
    legs = legs_of(meta)
    masks = [leg_mask(d, L["leg"]) for L in legs]
    elim = np.nanpercentile(np.abs(d["err_um"][d["leg"] >= 0]), 99.9) * 1.1 if (d["leg"] >= 0).any() else 50
    for w in range(n_win):
        x0, x1 = w * win_mm, min((w + 1) * win_mm, end)
        inner = outer[w // ncols, w % ncols].subgridspec(2, 1, hspace=0.05, height_ratios=[1, 1])
        a_err = fig.add_subplot(inner[0])
        a_out = fig.add_subplot(inner[1], sharex=a_err)
        worst_out = 0.0
        for L, m in zip(legs, masks):
            sel = m & (d["pos_mm"] >= x0) & (d["pos_mm"] < x1)
            if not sel.any():
                continue
            st = style_for(L)
            a_err.plot(d["pos_mm"][sel], d["err_um"][sel], lw=0.6, **st)
            a_out.plot(d["pos_mm"][sel], d["out_deg"][sel], lw=0.6, **st)
            worst_out = max(worst_out, np.max(np.abs(d["out_deg"][sel])))
        for x in np.arange(x0, x1 + 1e-6, PITCH_MM):     # bordes de vuelta de motor
            a_err.axvline(x, color=AXIS, lw=0.8)
            a_out.axvline(x, color=AXIS, lw=0.8)
        limit_lines(a_out, label=False)
        a_out.set_ylim(-OUT_LIMIT_DEG - 8, OUT_LIMIT_DEG + 8)
        a_err.set_ylim(-elim, elim)
        a_err.set_xlim(x0, x1)
        a_err.tick_params(labelbottom=False)
        warn = worst_out > 70
        a_err.set_title(f"{x0:g}–{x1:g} mm   máx |salida| {worst_out:.0f}°" + ("  ⚠" if warn else ""),
                        fontsize=9, color=CRITICAL if warn else INK)
        if w % ncols == 0:
            a_err.set_ylabel("error [µm]")
            a_out.set_ylabel("salida [° el.]")
        if w == 0:
            leg_legend(a_err, meta, loc="upper left", ncol=2)
    fig.suptitle(f"Carril en ventanas de {win_mm:g} mm ({win_mm / PITCH_MM:g} vueltas de motor, "
                 f"{win_mm / ENC_PERIOD_MM:.0f} períodos del encoder). Datos crudos; líneas grises = "
                 "borde de cada vuelta de motor; rojo = límite de la salida",
                 x=0.01, ha="left", fontsize=12, fontweight="bold", y=0.995)
    fig.subplots_adjust(left=0.04, right=0.99, top=0.975, bottom=0.02)
    out = base + "_ventanas.png"
    fig.savefig(out, dpi=80)
    plt.close(fig)
    return out


def fig_plegado(d, meta, base):
    keys = [("err_um", "error [µm]"), ("out_deg", "salida [° el.]"), ("vel_mm_s", "|velocidad| [mm/s]")]
    folds = [("pos_mm", lambda x: x % PITCH_MM, np.linspace(0, PITCH_MM, 121),
              "fase en la vuelta de motor [mm]"),
             ("enc_deg", lambda x: x, np.linspace(0, 360, 73),
              "ángulo crudo del encoder en el período [°]  (1 período = 2 imanes)")]
    fig, axs = plt.subplots(len(keys), 2, figsize=(16, 10), sharex="col", gridspec_kw=dict(hspace=0.1))
    for c, (src, f, edges, xlabel) in enumerate(folds):
        xc = 0.5 * (edges[1:] + edges[:-1])
        for r, (key, ylabel) in enumerate(keys):
            ax = axs[r, c]
            for L in legs_of(meta):
                m = leg_mask(d, L["leg"]) & d["cruise"]
                if not m.any():
                    continue
                y = d[key][m]
                if key == "vel_mm_s":
                    y = np.abs(y)
                mean, _, _ = binned(f(d[src][m]), y, edges)
                ax.plot(xc, mean, lw=1.6, **style_for(L))
            if key == "out_deg":
                limit_lines(ax, label=(c == 1))
                ax.set_ylim(-OUT_LIMIT_DEG - 8, OUT_LIMIT_DEG + 8)
            if c == 0:
                ax.set_ylabel(ylabel)
            if r == len(keys) - 1:
                ax.set_xlabel(xlabel)
        if c == 0:
            for x in np.arange(0, PITCH_MM + 1e-6, ENC_PERIOD_MM * 5):
                for r in range(len(keys)):
                    axs[r, 0].axvline(x, color=GRID, lw=0.5)
    axs[0, 0].set_title("Plegado sobre una vuelta de motor (5 mm)")
    axs[0, 1].set_title("Plegado sobre un período del encoder")
    leg_legend(axs[0, 1], meta, ncol=2)
    fig.suptitle("Formas que se repiten: media a velocidad de crucero de todo el carril",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.06, right=0.95, top=0.92, bottom=0.07)
    out = base + "_plegado.png"
    fig.savefig(out, dpi=100)
    plt.close(fig)
    return out


def analyze(d, meta, base, win_mm):
    derive(d, meta)
    rows, drops = summary(d, meta)
    print()
    print_summary(rows, drops, meta)
    with open(base + "_resumen.txt", "w") as fh:
        print_summary(rows, drops, meta, out=fh)
    outs = [fig_resumen(d, meta, rows, base), fig_tiempo(d, meta, base)]
    if (d["leg"] >= 0).any():
        outs += [fig_mapa(d, meta, base), fig_ventanas(d, meta, base, win_mm), fig_plegado(d, meta, base)]
    print("\ngraficos:")
    for o in outs:
        print("  " + os.path.abspath(o))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--cycles", type=int, default=2, help="cantidad de idas y vueltas")
    ap.add_argument("--end", type=float, default=255.0, help="punta lejana [mm] (maximo 255)")
    ap.add_argument("--feed", type=float, default=2.0, help="velocidad [mm/s] (el firmware limita a 2; para mas, --pre \"M203 X4\")")
    ap.add_argument("--decimation", type=int, default=2, help="M120 P: 1 = 2 kHz, 2 = 1 kHz")
    ap.add_argument("--settle", type=float, default=0.5, help="tiempo asentado para dar el tramo por terminado [s]")
    ap.add_argument("--dwell", type=float, default=1.0, help="reposo en cada punta [s]")
    ap.add_argument("--window-mm", type=float, default=10.0, help="ancho de las ventanas del grafico de ventanas")
    ap.add_argument("--pre", action="append", default=[],
                    help="comando a mandar antes de arrancar (se puede repetir), p.ej. --pre 'M58 V1 F0'")
    ap.add_argument("--tag", default="", help="sufijo para el nombre de los archivos")
    ap.add_argument("--from-file", help="re-graficar un .npz ya capturado")
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()

    if args.from_file:
        z = np.load(args.from_file, allow_pickle=False)
        d = {k: z[k] for k in z.files if k != "meta"}
        meta = json.loads(str(z["meta"]))
        analyze(d, meta, os.path.splitext(args.from_file)[0], args.window_mm)
        return

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out_dir = args.out_dir or os.path.join(root, "mediciones", datetime.date.today().isoformat())
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, "lazo_cerrado_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                        + (f"_{args.tag}" if args.tag else ""))

    d, meta, log, extra = capture(args)
    np.savez_compressed(base + ".npz", meta=json.dumps(meta), **d)
    with open(base + ".log", "w") as fh:
        fh.write("\n".join(log) + "\n")
    if extra.get("m121"):
        with open(base + "_m121.csv", "w") as fh:
            fh.write("\n".join(l for l in extra["m121"] if not l.startswith("ok")) + "\n")
    print(f"\n{len(d['t_s'])} muestras guardadas en {base}.npz")
    if len(d["t_s"]) > 10:
        analyze(d, meta, base, args.window_mm)
    sys.exit(0 if meta["ok"] else 1)


if __name__ == "__main__":
    main()
