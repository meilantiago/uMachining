#!/usr/bin/env python3
"""Prueba 2 con interferómetro: espectro del residuo de posición a velocidad constante.

Lo usa la simulación (simulaciones/prueba2_simulacion.py) y sirve igual para los
datos reales del Michelson. Pasos:

1. Señal del Michelson -> fase -> posición x_i(t).
   - cuadratura (dos detectores a 90°): ajuste de elipse (Heydemann) y atan2;
   - un detector: pasa-altos + transformada de Hilbert. Solo vale mientras el carro
     se mueve siempre en el mismo sentido (la franja nunca se detiene);
   - posición: si el equipo ya entrega x(t).
2. Se diezma a una frecuencia de análisis con filtro antialias.
3. Se busca el tramo de velocidad constante y se descartan rampas y transitorios.
4. Residuo = x − recta ajustada. Espectro de amplitud (Hann, con relleno de ceros).
5. Amplitud en cada frecuencia candidata (espacial: f = v/λ; temporal: fija) y
   búsqueda de picos que no estén en la lista.
6. Comparación x_i contra x_e por componente, y entre velocidades para los picos
   desconocidos (si se corren con v son espaciales; si no, temporales).

Uso con datos reales (CSV con encabezado):
    python tools/prueba2_analisis.py --interf michelson.csv --modo cuadratura \
        --encoder telemetria.csv --nombre v2
  michelson.csv: t_s, I1[, I2]  (o t_s, x_nm con --modo posicion)
  telemetria.csv: t_s, x_mm     (posición del encoder en unidades de carro)
"""
from __future__ import annotations

import argparse

import numpy as np
from scipy import ndimage, signal

LAMBDA_NM = 632.8   # HeNe en vacío. Cambiar si el láser del Michelson es otro.
N_AIRE = 1.000271   # aire a ~20 °C, 101,3 kPa, 50 % HR. Corregir con T, P, HR medidas.

# Períodos en el carro (mm). Husillo de 5 mm por vuelta.
PERIODOS_ESPACIALES_MM = {
    "husillo 1x": 5.0,
    "husillo 2x": 2.5,
    "encoder 1x (30/vuelta)": 5.0 / 30,
    "encoder 2x = imanes (60/vuelta)": 5.0 / 60,   # misma frecuencia: no se separan
    "paso motor 1x (200/vuelta)": 5.0 / 200,
    "paso motor 2x": 5.0 / 400,
}
FRECUENCIAS_FIJAS_HZ = {
    "oscilación del lazo (~214 Hz)": 214.0,
    "red eléctrica": 50.0,
}


# ---------------------------------------------------------------- demodulación
def ajustar_elipse(I1, I2, n_max=200_000):
    """Ajuste de cónica A x² + B xy + C y² + D x + E y = 1 (Heydemann).

    Modelo: I1 = x0 + a cos φ ;  I2 = y0 + b sin(φ + α).
    """
    idx = np.linspace(0, len(I1) - 1, min(n_max, len(I1))).astype(int)
    x, y = I1[idx], I2[idx]
    M = np.column_stack([x * x, x * y, y * y, x, y])
    A, B, C, D, E = np.linalg.lstsq(M, np.ones_like(x), rcond=None)[0]
    x0, y0 = np.linalg.solve([[2 * A, B], [B, 2 * C]], [-D, -E])
    k = 1.0 - (A * x0 * x0 + B * x0 * y0 + C * y0 * y0 + D * x0 + E * y0)
    A, B, C = A / k, B / k, C / k
    s = -B / (2.0 * np.sqrt(A * C))
    c2 = 1.0 - s * s
    return dict(x0=x0, y0=y0, a=1 / np.sqrt(A * c2), b=1 / np.sqrt(C * c2), alfa=np.arcsin(s))


def fase_cuadratura(I1, I2, elipse=None):
    e = elipse or ajustar_elipse(I1, I2)
    c = (I1 - e["x0"]) / e["a"]
    s = ((I2 - e["y0"]) / e["b"] - c * np.sin(e["alfa"])) / np.cos(e["alfa"])
    return np.unwrap(np.arctan2(s, c)), e


def fase_un_detector(I, fs, f_corte_hz):
    """Fase con un solo detector. f_corte debe quedar por debajo de la frecuencia
    de franja mínima (2·v_min/λ) y por encima de las variaciones de offset."""
    sos = signal.butter(4, f_corte_hz, "highpass", fs=fs, output="sos")
    return np.unwrap(np.angle(signal.hilbert(signal.sosfiltfilt(sos, I))))


def fase_a_mm(fase, lambda_nm=LAMBDA_NM, n_aire=N_AIRE):
    # Michelson: un período de franja = λ/2 de desplazamiento.
    return fase * lambda_nm / (4 * np.pi * n_aire) * 1e-6


def frecuencia_franja_hz(v_mm_s, lambda_nm=LAMBDA_NM, n_aire=N_AIRE):
    return 2 * n_aire * v_mm_s * 1e6 / lambda_nm


# ------------------------------------------------------------ preprocesamiento
def diezmar(t, x, fs_out):
    """Diezma con filtro antialias. Se saca la tendencia para no filtrar la rampa."""
    fs = 1.0 / np.median(np.diff(t))
    q = int(round(fs / fs_out))
    if q <= 1:
        return t, x
    p = np.polyfit(t - t[0], x, 1)
    r = x - np.polyval(p, t - t[0])
    rd = signal.resample_poly(r, 1, q)
    td = t[::q][: len(rd)]
    rd = rd[: len(td)]
    return td, rd + np.polyval(p, td - t[0])


def ventana_crucero(t, x, tol=0.05, margen_s=0.5, suavizado_s=0.5):
    """Índices del tramo más largo a velocidad constante, sin los bordes.

    La velocidad se suaviza (0,5 s) para que el ripple propio de la prueba, que
    puede ser de varios %, no corte la ventana."""
    fs = 1.0 / np.median(np.diff(t))
    n = max(3, int(suavizado_s * fs))
    v = np.gradient(ndimage.uniform_filter1d(x, n, mode="nearest"), t)
    vmax = np.percentile(np.abs(v), 95)
    v_cr = np.median(v[np.abs(v) > 0.5 * vmax])
    ok = np.abs(v - v_cr) < tol * abs(v_cr)
    d = np.diff(np.concatenate([[0], ok.astype(int), [0]]))
    ini, fin = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    j = np.argmax(fin - ini)
    m = int(margen_s * fs)
    return slice(ini[j] + m, fin[j] - m)


def residuo(t, x_mm):
    """Residuo respecto de la recta ajustada (nm) y velocidad ajustada (mm/s)."""
    tt = t - t[0]
    p = np.polyfit(tt, x_mm, 1)
    return (x_mm - np.polyval(p, tt)) * 1e6, p[0]


def espectro_amplitud(delta_nm, fs, relleno=4):
    """Amplitud de pico (nm) de cada componente senoidal. Hann + relleno de ceros
    para que el máximo de cada pico no dependa de dónde cae el bin."""
    n = len(delta_nm)
    w = signal.windows.hann(n)
    nfft = 1 << int(np.ceil(np.log2(relleno * n)))
    X = np.fft.rfft((delta_nm - delta_nm.mean()) * w, nfft)
    return np.fft.rfftfreq(nfft, 1 / fs), 2 * np.abs(X) / w.sum()


# --------------------------------------------------------------- componentes
def candidatos(v_mm_s):
    c = [dict(nombre=k, tipo="espacial", periodo_mm=p, f=abs(v_mm_s) / p)
         for k, p in PERIODOS_ESPACIALES_MM.items()]
    c += [dict(nombre=k, tipo="temporal", periodo_mm=None, f=f) for k, f in FRECUENCIAS_FIJAS_HZ.items()]
    return c


def _tolerancia(f, tipo, df):
    return max(3 * df, (0.005 if tipo == "espacial" else 0.05) * f)


def amplitud_en(f, amp, f0, tol, excl):
    """Máximo cerca de f0 y piso: mediana entre f0/3 y 3·f0, sin el lóbulo del pico."""
    sel = (f > f0 - tol) & (f < f0 + tol)
    if not sel.any():
        return np.nan, np.nan, np.nan
    i = np.flatnonzero(sel)[np.argmax(amp[sel])]
    ban = (f > f0 / 3) & (f < 3 * f0) & (np.abs(f - f0) > max(tol, excl))
    piso = np.median(amp[ban]) if ban.any() else np.nan
    return amp[i], f[i], piso


def picos_desconocidos(f, amp, df, conocidos, f_min=0.5, umbral=8.0, n_max=30, lobulo=10):
    """Picos que sobresalen umbral× del piso local y no coinciden con candidatos.

    Se ignora todo lo que cae a menos de `lobulo` resoluciones de un candidato o de
    un pico mayor ya aceptado: son lóbulos laterales de la ventana, no componentes."""
    dist = max(1, int(2 * df / (f[1] - f[0])))
    idx, _ = signal.find_peaks(amp, distance=dist)
    idx = idx[(f[idx] > f_min) & (f[idx] < 0.9 * f[-1])]
    idx = idx[np.argsort(amp[idx])[::-1][: 4 * n_max]]
    out = []
    for i in idx:
        f0 = f[i]
        if any(abs(f0 - c["f"]) < max(_tolerancia(c["f"], c["tipo"], df), lobulo * df) for c in conocidos):
            continue
        if any(abs(f0 - p["f"]) < lobulo * df for p in out):
            continue
        ban = (f > 0.8 * f0) & (f < 1.25 * f0)
        piso = np.median(amp[ban])
        if amp[i] > umbral * piso:
            out.append(dict(f=f0, amp=amp[i], piso=piso))
    return sorted(out, key=lambda p: -p["amp"])[:n_max]


def clasificar_sensores(Ai, pi, Ae, pe, k=5.0):
    di, de = Ai > k * pi, Ae > k * pe
    if di and de:
        r = Ai / Ae
        if r > 3:
            return "domina x_i"
        if r < 1 / 3:
            return "domina x_e"
        return "ambos, igual amplitud" if 0.8 < r < 1.25 else "ambos, distinta amplitud"
    return "solo x_i" if di else ("solo x_e" if de else "no detectado")


INTERPRETACION = {
    ("espacial", "ambos, igual amplitud"): "movimiento real de motor y carro (fuerza: cogging, torque). Confirmar que x_i/x_e ≈ 1 a todas las velocidades",
    ("espacial", "ambos, distinta amplitud"): "error del encoder repartido por el lazo (x_i = n·|T|, x_e = n·|S|) o fuerza más error",
    ("espacial", "solo x_i"): "error que el encoder no ve: husillo/guía, o no linealidad del encoder impresa por el lazo",
    ("espacial", "domina x_i"): "error del encoder impreso por el lazo (o del husillo), con parte por encima del ancho de banda",
    ("espacial", "solo x_e"): "artefacto del encoder por encima del ancho de banda del lazo",
    ("espacial", "domina x_e"): "artefacto del encoder por encima del ancho de banda del lazo",
    ("temporal", "ambos, igual amplitud"): "dinámica del lazo o resonancia del lado del motor",
    ("temporal", "ambos, distinta amplitud"): "dinámica del lazo o resonancia, con transmisión distinta al carro",
    ("temporal", "solo x_i"): "resonancia entre motor y carro, vibración de la guía o del montaje óptico",
    ("temporal", "domina x_i"): "resonancia entre motor y carro, vibración de la guía o del montaje óptico",
    ("temporal", "solo x_e"): "ruido o artefacto del encoder",
    ("temporal", "domina x_e"): "ruido o artefacto del encoder",
}


def ripple_velocidad(delta_nm, fs, f_max=500.0):
    """Ripple de velocidad (µm/s) del residuo, limitado a f_max."""
    sos = signal.butter(4, f_max, "lowpass", fs=fs, output="sos")
    return np.gradient(signal.sosfiltfilt(sos, delta_nm), 1 / fs) * 1e-3


# ------------------------------------------------------------------ análisis
def analizar_senal(t, x_mm, fs_analisis):
    t, x_mm = diezmar(t, x_mm, fs_analisis)
    w = ventana_crucero(t, x_mm)
    tw, xw = t[w], x_mm[w]
    d, v = residuo(tw, xw)
    fs = 1.0 / np.median(np.diff(tw))
    f, amp = espectro_amplitud(d, fs)
    return dict(t=tw, x=xw, delta=d, v=v, fs=fs, f=f, amp=amp, df=fs / len(d),
                T=tw[-1] - tw[0], L=abs(xw[-1] - xw[0]))


def analizar_corrida(t_i, x_i_mm, t_e, x_e_mm, fs_analisis=5000.0):
    """x_i y x_e con relojes independientes: no se necesita sincronismo."""
    ri = analizar_senal(t_i, x_i_mm, fs_analisis)
    re = analizar_senal(t_e, x_e_mm, fs_analisis)
    v = ri["v"]
    filas = []
    for c in candidatos(v):
        # Cada sensor con su propia velocidad medida (su propio reloj).
        fi = c["f"] if c["tipo"] == "temporal" else abs(ri["v"]) / c["periodo_mm"]
        fe = c["f"] if c["tipo"] == "temporal" else abs(re["v"]) / c["periodo_mm"]
        fila = dict(c, f=fi)
        for k, r, f0 in (("i", ri, fi), ("e", re, fe)):
            if f0 > 0.45 * r["fs"] or f0 < 2 / r["T"]:
                fila[f"A_{k}"], fila[f"piso_{k}"] = np.nan, np.nan
                continue
            a, _, p = amplitud_en(r["f"], r["amp"], f0, _tolerancia(f0, c["tipo"], r["df"]), 6 * r["df"])
            fila[f"A_{k}"], fila[f"piso_{k}"] = a, p
        if np.isnan(fila["A_e"]):
            fila["sensores"] = "fuera de banda del encoder" if not np.isnan(fila["A_i"]) else "fuera de banda"
        else:
            fila["sensores"] = clasificar_sensores(fila["A_i"], fila["piso_i"], fila["A_e"], fila["piso_e"])
        fila["interpretacion"] = INTERPRETACION.get((c["tipo"], fila["sensores"]), "")
        fila["dv_pct"] = 2 * np.pi * fi * fila["A_i"] * 1e-6 / abs(v) * 100
        filas.append(fila)
    cand = candidatos(v)
    rv = ripple_velocidad(ri["delta"], ri["fs"])
    return dict(
        v=v, interf=ri, encoder=re, componentes=filas,
        desconocidos_i=picos_desconocidos(ri["f"], ri["amp"], ri["df"], cand),
        desconocidos_e=picos_desconocidos(re["f"], re["amp"], re["df"], candidatos(re["v"])),
        ripple_rms_um_s=float(np.std(rv)),
        ripple_pp_pct=float(np.ptp(rv[len(rv) // 100: -len(rv) // 100]) / (abs(v) * 1e3) * 100),
    )


def escala_entre_reposos(t, x_mm, t_e, x_e_mm, reposo_s=0.5):
    """Error de escala x_i/x_e − 1 (ppm) entre el reposo inicial y el final.

    No necesita sincronismo: cada sensor da su propio Δx entre dos momentos en que
    el carro está quieto. Necesita que la demodulación no pierda franjas en las
    rampas (cuadratura, no un solo detector)."""
    def dx(t, x):
        a = x[t < t[0] + reposo_s].mean()
        b = x[t > t[-1] - reposo_s].mean()
        return b - a
    return (dx(t, x_mm) / dx(t_e, x_e_mm) - 1) * 1e6


def espectro_em(t_i, x_i_mm, t_e, x_e_mm, desfase_s=0.0, deriva_ppm=0.0, fs_analisis=5000.0):
    """e_m = x_e − x_i con sincronismo. Requiere conocer cómo se pasa del reloj del
    encoder al del interferómetro: t_interf = desfase + t_enc / (1 + deriva).

    e_m separa lo que las amplitudes solas no pueden: una fuerza mueve igual a los
    dos sensores y se cancela; un error del encoder n(x) queda entero, porque
    x_e − x_i = n·S + n·T = n."""
    ti, xi = diezmar(t_i, x_i_mm, fs_analisis)
    te = desfase_s + t_e / (1 + deriva_ppm * 1e-6)
    ok = (te > ti[0]) & (te < ti[-1])
    te, xe = te[ok], x_e_mm[ok]
    w = ventana_crucero(te, xe)
    em = (xe[w] - np.interp(te[w], ti, xi)) * 1e6
    tt = te[w] - te[w][0]
    em = em - np.polyval(np.polyfit(tt, em, 1), tt)
    fs = 1.0 / np.median(np.diff(te))
    f, amp = espectro_amplitud(em, fs)
    return dict(f=f, amp=amp, df=fs / len(em), em=em, x=xe[w])


def clasificar_entre_velocidades(picos_por_v, tol=0.02):
    """picos_por_v: {v: [f, ...]}. Devuelve lista de (v, f, tipo, λ_mm o None)."""
    out = []
    vs = list(picos_por_v)
    for v in vs:
        for f in picos_por_v[v]:
            tipo, lam = "una sola velocidad", None
            for v2 in vs:
                if v2 == v:
                    continue
                for f2 in picos_por_v[v2]:
                    if abs(f2 - f) < tol * f:
                        tipo = "temporal (fija en Hz)"
                    elif abs(f2 / v2 - f / v) < tol * f / v:
                        tipo, lam = "espacial (se corre con v)", v / f
            out.append((v, f, tipo, lam))
    return out


def tabla_componentes(res):
    lin = ["| Componente | f (Hz) | x_i (nm) | x_e (nm) | Sensores | Ripple de v (%) | Lectura |",
           "|---|---|---|---|---|---|---|"]
    for c in res["componentes"]:
        def fmt(a):
            return "—" if np.isnan(a) else f"{a:.1f}"
        lin.append(f"| {c['nombre']} | {c['f']:.2f} | {fmt(c['A_i'])} | {fmt(c['A_e'])} | "
                   f"{c['sensores']} | {fmt(c['dv_pct']) if c['sensores'] != 'no detectado' else '—'} | "
                   f"{c['interpretacion']} |")
    return "\n".join(lin)


# ----------------------------------------------------------------------- CLI
def _leer_csv(ruta):
    return np.genfromtxt(ruta, delimiter=",", names=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--interf", required=True)
    ap.add_argument("--modo", choices=["cuadratura", "un_detector", "posicion"], required=True)
    ap.add_argument("--encoder", required=True)
    ap.add_argument("--lambda-nm", type=float, default=LAMBDA_NM)
    ap.add_argument("--n-aire", type=float, default=N_AIRE)
    ap.add_argument("--sentido", type=int, choices=[1, -1], default=1,
                    help="sentido del movimiento (solo para --modo un_detector)")
    ap.add_argument("--fs-analisis", type=float, default=5000.0)
    a = ap.parse_args()

    d = _leer_csv(a.interf)
    t = d["t_s"]
    fs = 1 / np.median(np.diff(t))
    if a.modo == "posicion":
        x = d["x_nm"] * 1e-6
    elif a.modo == "cuadratura":
        fase, e = fase_cuadratura(d["I1"], d["I2"])
        print("Elipse:", {k: round(float(v), 4) for k, v in e.items()})
        x = fase_a_mm(fase, a.lambda_nm, a.n_aire)
    else:
        fase = fase_un_detector(d["I1"], fs, 100.0)
        x = a.sentido * fase_a_mm(fase, a.lambda_nm, a.n_aire)
    enc = _leer_csv(a.encoder)
    res = analizar_corrida(t, x, enc["t_s"], enc["x_mm"], a.fs_analisis)
    print(f"v = {res['v']:.4f} mm/s, ventana {res['interf']['L']:.1f} mm / {res['interf']['T']:.1f} s")
    print(tabla_componentes(res))
    print("Picos no esperados en x_i:", [(round(p["f"], 2), round(p["amp"], 2)) for p in res["desconocidos_i"]])
    print(f"Ripple de velocidad: {res['ripple_rms_um_s']:.2f} µm/s RMS, {res['ripple_pp_pct']:.1f} % pico a pico")


if __name__ == "__main__":
    main()
