"""Oscilacion en reposo (stick-slip) despues de mover: por configuracion, M55 + M17, ir a 12 mm y
volver a 11 mm a 1 mm/s, y grabar 8 s quieto. Uso: caza_reposo.py salida.npz '[["nombre","M55 ..."], ...]'"""
import sys, time, json
import numpy as np
sys.path.insert(0, "/home/laboratorio/Desktop/LAST DANCE (copy)/tools")
import closed_loop_test as T
out = sys.argv[1]; cfgs = json.loads(sys.argv[2])
link = T.Link("/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00", 921600)
time.sleep(0.5)
link.command("M120 S0", echo=False); link.wait_text(0.3); link.buf.clear()
link.command("G90", echo=False); link.command("M17", echo=False); link.command("M120 S1 P2", echo=False)
def wait(s):
    t0 = time.time()
    while time.time() - t0 < s: link.poll()
wait(0.5)
link.current_leg = -2
T.run_move(link, 11.0, 0.5, 0.3, "a 11 mm")
res = {}
try:
    for name, m55 in cfgs:
        link.command(m55, echo=False); link.command("M17", echo=False); wait(0.5)
        n0 = len(link.rows)
        ok = T.run_move(link, 12.0, 1.0, 0.2, f"{name} ida", abort_err_um=150) and \
             T.run_move(link, 11.0, 1.0, 0.2, f"{name} vuelta", abort_err_um=150)
        mv = np.array(link.rows[n0:], dtype=float)
        n1 = len(link.rows); wait(8.0)
        r = np.array(link.rows[n1:], dtype=float); r = r[len(r) // 4:]   # ultimos 6 s
        res[name] = r; res[name + "_mov"] = mv
        e = 1000 * (r[:, 1] - r[:, 2])
        dt = np.median(np.diff(r[:, 0])) * 1e-6
        F = np.fft.rfftfreq(len(e), dt); P = np.abs(np.fft.rfft((e - e.mean()) * np.hanning(len(e))))**2
        em = 1000 * np.abs(mv[:, 1] - mv[:, 2])
        print(f"{name:14s} {m55:32s} ok={int(ok)} reposo p-p {np.ptp(e):5.2f} um  desvio {e.std():5.3f} um  "
              f"pico {F[F>0.2][np.argmax(P[F>0.2])]:5.2f} Hz  |out| {np.degrees(np.abs(r[:,4])).max():4.1f}°   "
              f"mov max|err| {em.max():5.1f} um", flush=True)
        if not ok: link.command("M17", echo=False); T.run_move(link, 11.0, 0.5, 0.3, "rescate")
finally:
    link.command("M120 S0", echo=False); link.wait_text(0.3)
    np.savez_compressed(out, **res)
    link.close()
