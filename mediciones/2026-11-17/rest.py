import numpy as np, glob, json, sys, os
d = sys.argv[1]
order = [c["archivo"] for c in json.load(open(os.path.join(d, "barrido.json")))["configs"]]
print(f"{'config':18s} {'segs':>4s} {'desvio nm':>9s} {'p-p nm':>7s} {'150-260Hz':>9s} {'pico Hz':>7s}")
for f in order:
    z = np.load(os.path.join(d, f), allow_pickle=True)
    t = z["t_s"]; tg = z["target_mm"]; pos = z["pos_mm"]; leg = z["leg"]
    err = 1e6 * (tg - pos)   # nm
    # tramos quietos: objetivo constante
    still = np.r_[False, np.abs(np.diff(tg)) < 1e-9]
    # segmentos contiguos
    idx = np.where(still)[0]
    segs = np.split(idx, np.where(np.diff(idx) > 1)[0] + 1)
    stds, pps, fr, pk = [], [], [], []
    for s in segs:
        if len(s) < 700: continue
        s = s[400:]          # descartar 0,4 s de asentamiento
        e = err[s] - err[s].mean()
        stds.append(e.std()); pps.append(np.ptp(e))
        dt = np.median(np.diff(t[s]))
        F = np.fft.rfftfreq(len(e), dt); P = np.abs(np.fft.rfft(e * np.hanning(len(e))))**2
        band = (F > 150) & (F < 260); tot = P[F > 5].sum()
        fr.append(P[band].sum() / tot if tot else np.nan); pk.append(F[F > 5][np.argmax(P[F > 5])])
    name = f[4:-4]
    if stds:
        print(f"{name:18s} {len(stds):4d} {np.median(stds):9.1f} {np.median(pps):7.1f} {100*np.median(fr):8.0f}% {np.median(pk):7.0f}")
    else:
        print(name, "sin tramos quietos largos")
