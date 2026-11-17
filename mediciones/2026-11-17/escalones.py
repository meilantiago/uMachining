"""Respuesta a escalones chicos (10 nm, 100 nm, 1 um, 10 um) por configuracion de ganancias.
Uso: escalones.py salida.npz POS_MM '[["nombre","M55 ..."], ...]'
Por escalon: sobrepaso [% del escalon], t90 (llega al 90 %), t_asent (ultima salida de la banda
max(20 % del escalon, 4 nm)), error final medio (ultimos 0,5 s)."""
import sys, time, json
import numpy as np
sys.path.insert(0, "/home/laboratorio/Desktop/LAST DANCE (copy)/tools")
import closed_loop_test as T
out = sys.argv[1]; P = float(sys.argv[2]); cfgs = json.loads(sys.argv[3])
# (desplazamiento acumulado desde P en um, tiempo de espera s)
SEQ = [(0.01, 3), (0.02, 3), (0.03, 3), (0.13, 3), (0.03, 3), (1.03, 3), (0.03, 3), (10.03, 2.5), (0.03, 2.5)]
link = T.Link("/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00", 921600)
time.sleep(0.5)
link.command("M120 S0", echo=False); link.wait_text(0.3); link.buf.clear()
link.command("G90", echo=False); link.command("M17", echo=False); link.command("M120 S1 P1", echo=False)
def wait(s):
    t0 = time.time()
    while time.time() - t0 < s: link.poll()
wait(0.5)
link.current_leg = -2
T.run_move(link, P, 0.5, 0.3, f"a {P} mm")
res = {}
def analyze(r, prev, dest):
    t = np.cumsum(np.r_[0, np.diff(r[:, 0]) % 2**32]) * 1e-6; tg = r[:, 1]; pos = r[:, 2]   # t_us de 32 bits
    S = (dest - prev) * 1e6                              # nm
    k0 = np.argmax(np.abs(tg - prev) > 1e-9) if np.any(np.abs(tg - prev) > 1e-9) else 0
    t = t - t[k0]
    x = (pos - prev) * 1e6                               # nm desde el nivel anterior
    sgn = np.sign(S)
    over = max(0.0, np.max(sgn * (x[k0:] - S))) / abs(S) * 100
    reach = np.where(sgn * x[k0:] >= 0.9 * abs(S))[0]
    t90 = t[k0:][reach[0]] if len(reach) else np.nan
    band = max(0.2 * abs(S), 4.0)
    outb = np.where(np.abs(x[k0:] - S) > band)[0]
    tset = t[k0:][outb[-1]] if len(outb) else 0.0
    fin = x[t > t[-1] - 0.5].mean() - S
    return S, over, t90, tset, fin, t[-1]
try:
    for name, m55 in cfgs:
        for c in m55.split(";"):                         # varios comandos separados por ';'
            link.command(c.strip(), echo=False)
        link.command("M17", echo=False)
        T.run_move(link, P, 0.5, 0.3, "nivel")
        wait(2.0)
        prev = link.rows[-1][1]
        print(f"\n== {name}: {m55}")
        rows_cfg = []
        for off_um, hold in SEQ:
            dest = P + off_um * 1e-3
            n0 = len(link.rows)
            link.command(f"G1 X{dest:.7f} F2", echo=False)
            wait(hold)
            r = np.array(link.rows[n0:], dtype=float)
            S, over, t90, tset, fin, dur = analyze(r, prev, dest)
            flag = "  (no se asento)" if tset > dur - 0.6 else ""
            print(f"  paso {S:+9.1f} nm  sobrepaso {over:6.0f} %  t90 {1000*t90:7.1f} ms  asentado {1000*tset:7.0f} ms  "
                  f"error final {fin:+6.1f} nm{flag}", flush=True)
            rows_cfg.append(r)
            prev = dest
        res[name] = np.concatenate(rows_cfg)
finally:
    link.command("M120 S0", echo=False); link.wait_text(0.3)
    np.savez_compressed(out, **res)
    link.close()
