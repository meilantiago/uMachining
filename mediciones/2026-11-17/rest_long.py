"""Reposo largo por configuracion: M55, M17 en el lugar, esperar, grabar N s de telemetria a 2 kHz."""
import sys, time, json
import numpy as np
sys.path.insert(0, "/home/laboratorio/Desktop/LAST DANCE (copy)/tools")
import closed_loop_test as T
out = sys.argv[1]; pos_mm = float(sys.argv[2]); cfgs = json.loads(sys.argv[3])
link = T.Link("/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00", 921600)
time.sleep(0.5)
link.command("M120 S0", echo=False); link.wait_text(0.3); link.buf.clear()
link.command("G90", echo=False)
link.command("M17", echo=False)
link.command("M120 S1 P1", echo=False)
def wait(s):
    t0 = time.time()
    while time.time() - t0 < s: link.poll()
wait(0.5)
link.current_leg = -2
T.run_move(link, pos_mm, 0.5, 0.5, f"a {pos_mm} mm")
res = {}
try:
    for name, m55 in cfgs:
        for c in m55.split(";"): link.command(c.strip(), echo=False)
        link.command("M17", echo=False)
        wait(3.0)
        n0 = len(link.rows)
        wait(5.0)
        r = np.array(link.rows[n0:], dtype=float)
        res[name] = r
        e = 1e6 * (r[:, 1] - r[:, 2]); e -= e.mean()
        dt = np.median(np.diff(r[:, 0])) * 1e-6
        F = np.fft.rfftfreq(len(e), dt); P = np.abs(np.fft.rfft(e * np.hanning(len(e))))**2
        band = (F > 150) & (F < 260); tot = P[F > 2].sum()
        print(f"{name:18s} {m55:34s} desvio {e.std():6.2f} nm  p-p {np.ptp(e):6.1f} nm  150-260 Hz {100*P[band].sum()/tot:3.0f}%  "
              f"pico {F[F>2][np.argmax(P[F>2])]:5.0f} Hz  |out| max {np.degrees(np.abs(r[:,4])).max():4.1f}°  dt {dt*1e3:.2f} ms", flush=True)
finally:
    link.command("M120 S0", echo=False); link.wait_text(0.3)
    link.command("M55 A120 B0 C0.09 D30 F0.004", echo=False); link.command("M55 H60 V0.5", echo=False)   # valores de hw_config.h
    np.savez_compressed(out, **{k: v for k, v in res.items()})
    link.close()
