#!/usr/bin/env python3
"""Prueba 4 con interferómetro: respuesta en frecuencia (Bode) del carro.

Se comanda x_c = x0 + A·sen(2πft) a una frecuencia por vez. El RP2040 registra
x_c y x_e (encoder) con su reloj; el interferómetro registra x_i con el suyo.
Para cada frecuencia se ajusta una senoide (más armónicos) por mínimos cuadrados
y se calculan:

  G_e = X_e / X_c   lazo visto por el encoder (mismo reloj: no necesita sincronismo)
  G_i = X_i / X_c   lo que hace el carro de verdad (necesita sincronismo para la fase)
  G_m = X_i / X_e   transmisión mecánica motor -> carro (husillo, tuerca, acople)

Sincronismo entre los dos relojes: t_interf = desfase + t_rp / (1 + deriva).
  - "hardware": desfase y deriva conocidos (pulso de disparo + corrección).
  - "marcas": se estiman con dos escalones cortos al principio y al final.
  - "una_marca": solo el desfase (deriva = 0).
La magnitud de G_i y G_m no depende del sincronismo; la fase sí.

Lo usa simulaciones/prueba4_simulacion.py y sirve para los datos reales.
"""
from __future__ import annotations

import numpy as np

import prueba2_analisis as p2   # demodulación del Michelson y diezmado


# ------------------------------------------------------------- sincronismo
def a_reloj_interf(t_rp, desfase_s, deriva_ppm):
    return desfase_s + t_rp / (1 + deriva_ppm * 1e-6)


def a_reloj_rp(t_i, desfase_s, deriva_ppm):
    return (t_i - desfase_s) * (1 + deriva_ppm * 1e-6)


def _cruce_mitad(t, x, desde, hasta):
    """Instante en que la marca pasa por la mitad de su altura (cada señal con su escala)."""
    sel = (t >= desde) & (t <= hasta)
    tt, xx = t[sel], x[sel]
    base = np.median(xx[: max(5, len(xx) // 20)])
    d = xx - base
    alto = d[np.argmax(np.abs(d))]
    idx = np.flatnonzero(d / alto > 0.5)
    if len(idx) == 0 or abs(alto) < 1e-4:
        raise ValueError("no se encontró la marca")
    return tt[idx[0]]


def _refinar(t_i, x_i, t_e, x_e, tr0, ti0, ventana_s, busqueda_s=5e-3, paso_s=5e-6):
    """Retardo que mejor superpone x_i sobre a + b·x_e alrededor de la marca.

    a y b se ajustan en cada retardo: el carro no tiene por qué moverse lo mismo
    que el motor (acople, fricción), solo con la misma forma."""
    sel = (t_e >= tr0 - 0.15) & (t_e <= tr0 + ventana_s)
    te, xe = t_e[sel], x_e[sel]
    M = np.column_stack([np.ones_like(xe), xe])
    deltas = np.arange(-busqueda_s, busqueda_s, paso_s)
    err = np.empty(len(deltas))
    for k, d in enumerate(deltas):
        xi = np.interp(te - tr0 + ti0 + d, t_i, x_i)
        coef, res, *_ = np.linalg.lstsq(M, xi, rcond=None)
        err[k] = res[0] if len(res) else np.sum((M @ coef - xi) ** 2)
    k = int(np.clip(np.argmin(err), 1, len(err) - 2))
    y0, y1, y2 = err[k - 1], err[k], err[k + 1]
    den = y0 - 2 * y1 + y2
    sub = 0.5 * (y0 - y2) / den if den != 0 else 0.0
    return ti0 + deltas[k] + sub * paso_s


def estimar_sincronismo(t_i, x_i, t_e, x_e, marcas_rp, ventana_s=0.6, una_sola=False):
    """Desfase y deriva a partir de escalones cortos (marcas) en x_c.

    marcas_rp: tiempos (reloj del RP2040) donde arranca cada marca. Conviene que la
    marca sea lo bastante grande para que el carro deslice (decenas de µm).
    Devuelve (desfase_s, deriva_ppm)."""
    pares = []
    for m in (marcas_rp[:1] if una_sola else marcas_rp):
        tr = _cruce_mitad(t_e, x_e, m - 0.2, m + ventana_s)
        if not pares:
            ti = _cruce_mitad(t_i, x_i, t_i[0], t_i[0] + (m + ventana_s) * 1.01 + 1.0)
        else:
            ti1, tr1 = pares[0]
            ti_aprox = ti1 + (tr - tr1)
            ti = _cruce_mitad(t_i, x_i, ti_aprox - 0.3, ti_aprox + 0.3)
        ti = _refinar(t_i, x_i, t_e, x_e, tr, ti, ventana_s)
        pares.append((ti, tr))
    if len(pares) == 1:
        ti, tr = pares[0]
        return ti - tr, 0.0
    (ti1, tr1), (ti2, tr2) = pares[0], pares[-1]
    b = (ti2 - ti1) / (tr2 - tr1)
    return ti1 - b * tr1, (1 / b - 1) * 1e6

# -------------------------------------------------------- ajuste por segmento
def ajuste_senoidal(t, y, f, armonicos=3):
    """y ≈ Σ Re{X_h e^{j2πhft}} + a + b·t. Devuelve [X_1, X_2, ...] (complejos)."""
    t0 = t - t.mean()
    cols = []
    for h in range(1, armonicos + 1):
        w = 2 * np.pi * h * f * t
        cols += [np.cos(w), np.sin(w)]
    cols += [np.ones_like(t), t0, t0**2]
    coef = np.linalg.lstsq(np.column_stack(cols), y, rcond=None)[0]
    return np.array([coef[2 * h] - 1j * coef[2 * h + 1] for h in range(armonicos)])


def analizar_barrido(segmentos, t_rp, x_c, x_e, t_i, x_i, desfase_s, deriva_ppm):
    """segmentos: lista de dict(f, t0, t1) en reloj del RP2040 (ventana de medición).

    t_rp, x_c, x_e: telemetría (mm). t_i, x_i: interferómetro (mm, ya demodulado)."""
    t_i_rp = a_reloj_rp(t_i, desfase_s, deriva_ppm)
    filas = []
    for s in segmentos:
        f = s["f"]
        se = (t_rp >= s["t0"]) & (t_rp < s["t1"])
        si = (t_i_rp >= s["t0"]) & (t_i_rp < s["t1"])
        Xc = ajuste_senoidal(t_rp[se], x_c[se], f)
        Xe = ajuste_senoidal(t_rp[se], x_e[se], f)
        Xi = ajuste_senoidal(t_i_rp[si], x_i[si], f)
        Ge, Gi, Gm = Xe[0] / Xc[0], Xi[0] / Xc[0], Xi[0] / Xe[0]
        filas.append(dict(
            f=f, A_c_nm=abs(Xc[0]) * 1e6, A_i_nm=abs(Xi[0]) * 1e6,
            Ge=Ge, Gi=Gi, Gm=Gm,
            thd_i=float(np.sqrt(np.sum(np.abs(Xi[1:]) ** 2)) / abs(Xi[0])),
        ))
    return filas


def db(g):
    return 20 * np.log10(np.abs(g))


def fase_grad(g):
    return np.degrees(np.angle(g))


def ancho_de_banda(filas, clave="Gi"):
    """Primera frecuencia donde |G| cae 3 dB respecto de la frecuencia más baja."""
    f = np.array([r["f"] for r in filas])
    g = np.array([db(r[clave]) for r in filas])
    ref = g[0]
    for k in range(1, len(f)):
        if g[k] < ref - 3:
            # interpolación en log f
            a = (ref - 3 - g[k - 1]) / (g[k] - g[k - 1])
            return float(10 ** (np.log10(f[k - 1]) + a * (np.log10(f[k]) - np.log10(f[k - 1]))))
    return None


def ajustar_resonancia(filas, f_min=20.0):
    """|G_m| ≈ g0 / |1 − (f/fr)²| por debajo de la resonancia. Devuelve (g0, fr).

    Sirve aunque el pico quede por encima de la última frecuencia medida: la
    curvatura de |G_m| ya dice dónde está."""
    f = np.array([r["f"] for r in filas if r["f"] >= f_min])
    g = np.array([abs(r["Gm"]) for r in filas if r["f"] >= f_min])
    # 1/|G| = (1 − f²/fr²)/g0  ->  lineal en f²
    A = np.column_stack([np.ones_like(f), f**2])
    c0, c1 = np.linalg.lstsq(A, 1 / g, rcond=None)[0]
    g0 = 1 / c0
    fr = np.sqrt(-c0 / c1) if c1 < 0 else np.inf
    return float(g0), float(fr)


def tabla(filas):
    lin = ["| f (Hz) | A en el carro (nm) | |G_e| (dB) | ∠G_e (°) | |G_i| (dB) | ∠G_i (°) | |G_m| | ∠G_m (°) | THD x_i (%) |",
           "|---|---|---|---|---|---|---|---|---|"]
    for r in filas:
        lin.append(f"| {r['f']:g} | {r['A_i_nm']:.0f} | {db(r['Ge']):+.2f} | {fase_grad(r['Ge']):+.1f} | "
                   f"{db(r['Gi']):+.2f} | {fase_grad(r['Gi']):+.1f} | {abs(r['Gm']):.3f} | "
                   f"{fase_grad(r['Gm']):+.2f} | {100 * r['thd_i']:.1f} |")
    return "\n".join(lin)
