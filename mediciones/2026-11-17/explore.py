import sys, time, json
import numpy as np
sys.path.insert(0, "/home/laboratorio/Desktop/LAST DANCE (copy)/tools")
import closed_loop_test as T
out_path = sys.argv[1]; moves = json.loads(sys.argv[2]); m55 = sys.argv[3] if len(sys.argv) > 3 else None
link = T.Link("/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6632C8593188B33-if00", 921600)
time.sleep(0.5)
link.command("M120 S0", echo=False); link.wait_text(0.3); link.buf.clear()
link.command("G90", echo=False)
if m55: link.command(m55)
link.command("M17")
link.command("M120 S1 P2", echo=False)
t0=time.time()
while time.time()-t0 < 0.5: link.poll()
results = []
try:
    for k, (dest, feed) in enumerate(moves):
        link.current_leg = k
        ok = T.run_move(link, dest, feed, 0.5, f"G1 X{dest} F{feed}", abort_err_um=150)
        results.append(ok)
        if not ok:
            st = link.command("M119", echo=False); print("\n".join(st))
            break
finally:
    link.command("M120 S0", echo=False); link.wait_text(0.3)
    r = np.array(link.rows, dtype=float); lg = np.array(link.leg)
    np.savez_compressed(out_path, rows=r, leg=lg)
    link.close()
print("results", results)
