#!/usr/bin/env python3
"""Barrido de caracterizacion del encoder MT6835 (requiere el main.cpp temporal de barrido).

Manda 'G56', captura las muestras que manda la placa mientras el motor gira en lazo
abierto (2 vueltas ida + 2 vuelta), guarda un CSV por corrida en mediciones/AAAA-MM-DD/ y genera
un PNG con los graficos.

Uso:
    python3 tools/encoder_sweep.py                      # corre un barrido nuevo
    python3 tools/encoder_sweep.py --from-csv archivo   # re-grafica un CSV ya guardado

Cerrar antes cualquier otro programa que tenga el puerto serie abierto (monitor de
PlatformIO, o Teleplot conectado al puerto). Para ver en vivo, dejar Teleplot abierto SIN
conectarlo al puerto: el script le reenvia los datos por UDP (puerto 47269).
"""
import argparse
import datetime
import os
import time
import warnings

import numpy as np

warnings.filterwarnings("ignore", message="Unable to import Axes3D")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

COLS = ["t_s", "dir", "field_rad", "motor_deg", "x_mm", "raw", "enc_deg",
        "ideal_deg", "error_deg", "overspeed", "weak", "undervolt", "crc_err"]


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


class TeleplotUDP:
    """Reenvia datos a Teleplot por UDP (Teleplot escucha en el puerto 47269), asi se
    puede ver en vivo sin que Teleplot tenga que abrir el puerto serie."""

    def __init__(self, host, port, pole_pairs, cpr, decimation):
        import socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)
        self.pole_pairs, self.cpr, self.decimation = pole_pairs, cpr, decimation
        self.count = 0
        self.field0 = None

    def send(self, row):
        self.count += 1
        if self.count % self.decimation:
            return
        _, direction, field, raw, status = row
        if self.field0 is None:
            self.field0 = field
        motor_deg = np.degrees((field - self.field0) / self.pole_pairs)
        msg = (f"raw_angle_deg:{raw / self.cpr * 360.0:.3f}\n"
               f"motor_deg:{motor_deg:.3f}\n"
               f"sentido:{direction}\n"
               f"campo_debil:{(status >> 1) & 1}\n"
               f"overspeed:{status & 1}\n"
               f"error_crc:{(status >> 3) & 1}\n")
        try:
            self.sock.sendto(msg.encode(), self.addr)
        except OSError:
            pass


def capture(port, baud, timeout_s, teleplot=None):
    import serial
    rows, meta, end = [], "", ""
    with serial.Serial(port, baud, timeout=0.5) as ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.write(b"G56\n")
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            line = ser.readline().decode(errors="replace").strip()
            if not line:
                continue
            if line.startswith("D,"):
                p = line.split(",")
                row = [int(p[1]), int(p[2]), float(p[3]), int(p[4]), int(p[5])]
                rows.append(row)
                if teleplot:
                    teleplot.send(row)
            elif line.startswith("META"):
                meta = line
            elif line.startswith("END"):
                end = line
                break
            else:
                print("  " + line)
    return np.array(rows, dtype=float), meta, end


def build_table(raw_rows, pole_pairs, cpr, magnets, pitch_mm):
    t_us, direction, field, raw, status = raw_rows.T
    motor_rad = (field - field[0]) / pole_pairs
    periods = magnets / 2.0
    enc = raw / cpr * 360.0
    phase_cmd = np.degrees(motor_rad) * periods          # angulo esperado del campo, sin offset

    # sentido y offset de la relacion encoder <-> posicion comandada
    best = None
    for s in (+1.0, -1.0):
        r = np.radians(enc - s * phase_cmd)
        strength = np.abs(np.mean(np.exp(1j * r)))
        if best is None or strength > best[0]:
            best = (strength, s)
    s = best[1]
    ideal = np.zeros_like(enc)
    for d in (+1, -1):
        m = direction == d
        off = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(enc[m] - s * phase_cmd[m])))))
        ideal[m] = s * phase_cmd[m] + off
    error = wrap180(enc - ideal)

    st = status.astype(int)
    circumference = magnets * pitch_mm
    table = np.column_stack([
        t_us * 1e-6, direction, field, np.degrees(motor_rad), motor_rad / (2 * np.pi) * circumference,
        raw, enc, wrap180(ideal) % 360.0, error,
        st & 1, (st >> 1) & 1, (st >> 2) & 1, (st >> 3) & 1])
    return table, s


def analyze(table, periods, title, png_path):
    T = {c: table[:, i] for i, c in enumerate(COLS)}
    n = len(table)
    fwd = T["dir"] > 0
    bwd = ~fwd

    d_enc = wrap180(np.diff(T["enc_deg"]))
    d_ideal = wrap180(np.diff(T["ideal_deg"]))
    jumps = np.abs(wrap180(d_enc - d_ideal)) > 45.0
    jumps &= (np.diff(T["dir"]) == 0)

    print(f"muestras: {n}")
    print(f"campo debil: {int(T['weak'].sum())} ({100*T['weak'].mean():.1f} %)   "
          f"overspeed: {int(T['overspeed'].sum())}   subtension: {int(T['undervolt'].sum())}   "
          f"error CRC: {int(T['crc_err'].sum())}")
    print(f"saltos (> 45 grados respecto de lo esperado entre muestras seguidas): {int(jumps.sum())}")
    for name, m in (("ida", fwd), ("vuelta", bwd)):
        e = T["error_deg"][m]
        print(f"error vs ideal lineal ({name}): rms {np.sqrt(np.mean(e**2)):.1f} grados encoder, "
              f"max {np.max(np.abs(e)):.1f}  (= {np.sqrt(np.mean(e**2))/periods:.2f} grados de eje rms)")
    unw = np.unwrap(np.radians(T["enc_deg"][fwd]))
    slope = np.polyfit(np.radians(T["motor_deg"][fwd]), unw, 1)[0]
    print(f"periodos de encoder por vuelta (ida, sin corregir saltos): {abs(slope):.2f}")

    phase = T["enc_deg"]                     # fase medida dentro del periodo
    mech = T["motor_deg"] % 360.0

    fig, ax = plt.subplots(4, 2, figsize=(16, 18))
    fig.suptitle(title)

    # 1) como el grafico de la simulacion: angulo vs posicion a lo largo de la rueda
    a = ax[0, 0]
    cruise = fwd & (T["t_s"] > T["t_s"][fwd][0] + 1.0)
    x0 = T["x_mm"][cruise][0] if cruise.any() else 0.0
    w = cruise & (T["x_mm"] >= x0) & (T["x_mm"] <= x0 + 12.0)
    a.plot(T["x_mm"][w], wrap180(T["enc_deg"][w]), ".", ms=2, label="medido")
    ideal_w = wrap180(T["ideal_deg"][w])
    ideal_w[1:][np.abs(np.diff(ideal_w)) > 180] = np.nan   # no unir la linea donde da la vuelta
    a.plot(T["x_mm"][w], ideal_w, "-", lw=0.8, label="ideal lineal")
    a.plot(T["x_mm"][w & (T["weak"] > 0)], wrap180(T["enc_deg"][w & (T["weak"] > 0)]), "rx", ms=5, label="campo debil")
    a.set_xlabel("posicion a lo largo de la rueda [mm]"); a.set_ylabel("angulo del campo medido [grados]")
    a.set_title("Rotacion del campo medida (2 periodos, ida)"); a.legend()

    # 2) barrido completo
    a = ax[0, 1]
    a.plot(T["motor_deg"][fwd], T["enc_deg"][fwd], ".", ms=1, label="ida")
    a.plot(T["motor_deg"][bwd], T["enc_deg"][bwd], ".", ms=1, label="vuelta")
    wk = T["weak"] > 0
    a.plot(T["motor_deg"][wk], T["enc_deg"][wk], "rx", ms=3, label="campo debil")
    a.set_xlabel("posicion comandada del motor [grados]"); a.set_ylabel("angulo crudo [grados encoder]")
    a.set_title("Angulo crudo en todo el barrido"); a.legend(markerscale=4)

    # 3) error vs posicion
    a = ax[1, 0]
    a.plot(T["motor_deg"][fwd], T["error_deg"][fwd], ".", ms=1, label="ida")
    a.plot(T["motor_deg"][bwd], T["error_deg"][bwd], ".", ms=1, label="vuelta")
    a.set_xlabel("posicion comandada del motor [grados]"); a.set_ylabel("error vs ideal [grados encoder]")
    a.set_title("No linealidad a lo largo del barrido (incluye el atraso del rotor)"); a.legend(markerscale=4)

    # 4) error vs fase dentro del periodo
    a = ax[1, 1]
    a.plot(T["ideal_deg"][fwd], T["error_deg"][fwd], ".", ms=1, label="ida")
    a.plot(T["ideal_deg"][bwd], T["error_deg"][bwd], ".", ms=1, label="vuelta")
    a.set_xlabel("fase ideal dentro del periodo [grados]"); a.set_ylabel("error [grados encoder]")
    a.set_title("Forma de la no linealidad dentro de un periodo"); a.legend(markerscale=4)

    # 5) campo debil / overspeed segun la fase dentro del periodo
    a = ax[2, 0]
    bins = np.linspace(0, 360, 37)
    idx = np.clip(np.digitize(T["ideal_deg"], bins) - 1, 0, 35)
    frac_w = [T["weak"][idx == k].mean() if np.any(idx == k) else 0 for k in range(36)]
    frac_o = [T["overspeed"][idx == k].mean() if np.any(idx == k) else 0 for k in range(36)]
    a.bar(bins[:-1], np.array(frac_w) * 100, width=10, align="edge", label="campo debil")
    a.bar(bins[:-1], np.array(frac_o) * 100, width=10, align="edge", alpha=0.6, label="overspeed")
    a.set_xlabel("fase ideal dentro del periodo [grados]"); a.set_ylabel("% de muestras")
    a.set_title("Donde aparece el campo debil dentro de cada periodo"); a.legend()

    # 6) campo debil segun la posicion en la vuelta (localiza imanes)
    a = ax[2, 1]
    bins_m = np.linspace(0, 360, 61)
    idx = np.clip(np.digitize(mech, bins_m) - 1, 0, 59)
    frac = [T["weak"][idx == k].mean() if np.any(idx == k) else 0 for k in range(60)]
    a.bar(bins_m[:-1], np.array(frac) * 100, width=6, align="edge")
    a.set_xlabel("posicion en la vuelta del motor [grados] (1 barra = 1 iman)")
    a.set_ylabel("% de muestras con campo debil"); a.set_title("Campo debil a lo largo de la vuelta")

    # 7) saltos entre muestras
    a = ax[3, 0]
    a.plot(T["t_s"][1:], wrap180(d_enc - d_ideal), lw=0.5)
    a.plot(T["t_s"][1:][jumps], wrap180(d_enc - d_ideal)[jumps], "rx", ms=4, label="salto")
    a.set_xlabel("tiempo [s]"); a.set_ylabel("paso medido - esperado [grados]")
    a.set_title("Saltos de la lectura entre muestras seguidas"); a.legend()

    # 8) bits de estado en el tiempo
    a = ax[3, 1]
    for k, c in enumerate(["overspeed", "weak", "undervolt", "crc_err"]):
        m = T[c] > 0
        a.plot(T["t_s"][m], np.full(m.sum(), k), "|", ms=8)
    a.set_yticks(range(4)); a.set_yticklabels(["overspeed", "campo debil", "subtension", "error CRC"])
    a.set_xlabel("tiempo [s]"); a.set_title("Bits de estado del chip")

    for a in ax.flat:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=85)
    print(f"graficos en {os.path.abspath(png_path)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--magnets", type=int, default=60, help="cantidad de imanes de la rueda")
    ap.add_argument("--pitch", type=float, default=3.0, help="paso entre imanes [mm]")
    ap.add_argument("--pole-pairs", type=int, default=50, help="pares de polos del motor")
    ap.add_argument("--from-csv", help="re-graficar un CSV ya guardado")
    ap.add_argument("--no-teleplot", action="store_true", help="no reenviar los datos a Teleplot por UDP")
    ap.add_argument("--teleplot-port", type=int, default=47269)
    ap.add_argument("--teleplot-decimation", type=int, default=5,
                    help="reenviar 1 de cada N muestras a Teleplot (1 = todas, 1 kHz)")
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mediciones",
                                                     datetime.date.today().isoformat()),
                    help="carpeta de salida (por defecto mediciones/AAAA-MM-DD, la del dia)")
    args = ap.parse_args()

    periods = args.magnets / 2.0
    if args.from_csv:
        table = np.loadtxt(args.from_csv, delimiter=",", skiprows=1)
        base = os.path.splitext(args.from_csv)[0]
    else:
        teleplot = None
        if not args.no_teleplot:
            teleplot = TeleplotUDP("127.0.0.1", args.teleplot_port, args.pole_pairs, 2 ** 21,
                                   args.teleplot_decimation)
            print(f"Reenviando a Teleplot por UDP (127.0.0.1:{args.teleplot_port}). "
                  "En Teleplot NO abrir el puerto serie.")
        raw_rows, meta, end = capture(args.port, args.baud, args.timeout, teleplot)
        if len(raw_rows) < 100:
            print("No llegaron muestras. Esta cargado el main.cpp de barrido?")
            return
        print(meta)
        print(end)
        cpr = 2 ** 21
        table, sign = build_table(raw_rows, args.pole_pairs, cpr, args.magnets, args.pitch)
        os.makedirs(args.outdir, exist_ok=True)
        base = os.path.join(args.outdir, "encoder_sweep_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        np.savetxt(base + ".csv", table, delimiter=",", header=",".join(COLS), comments="",
                   fmt=["%.6f", "%d", "%.5f", "%.4f", "%.4f", "%d", "%.4f", "%.4f", "%.4f",
                        "%d", "%d", "%d", "%d"])
        print(f"CSV guardado en {os.path.abspath(base)}.csv")

    analyze(table, periods, os.path.basename(base), base + ".png")


if __name__ == "__main__":
    main()
