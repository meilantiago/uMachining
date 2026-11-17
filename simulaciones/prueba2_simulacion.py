#!/usr/bin/env python3
"""Simulación de la prueba 2 (espectro de velocidad y ripple) con un Michelson.

Genera lo que esperaríamos medir: la señal cruda de los fotodetectores del
Michelson y la telemetría del encoder, con relojes independientes. Después le
aplica el mismo análisis que se usaría con los datos reales
(tools/prueba2_analisis.py) y compara lo recuperado con lo que se puso.

Casos:
  A. Cuadratura (dos detectores) a 1, 2 y 5 mm/s.    <- caso principal
  B. Un solo detector a 2 mm/s.
  C. Cuadratura a 5 mm/s muestreada a 25 kS/s (por debajo de Nyquist de franja).

Salida: simulaciones/prueba2/ (figuras y RESULTADOS.md).
Todos los supuestos están en el bloque de parámetros y en RESULTADOS.md.
"""
from __future__ import annotations

import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import prueba2_analisis as an  # noqa: E402

SALIDA = pathlib.Path(__file__).resolve().parent / "prueba2"
RNG = np.random.default_rng(20261117)

# ======================================================================
# Parámetros. "medido" = sale del repo; "supuesto" = a confirmar en el INTI.
# ======================================================================
VELOCIDADES = [1.0, 2.0, 5.0]       # mm/s
ACEL = 5.0                          # mm/s², DEFAULT_MAX_ACCEL_MM_S2 (medido)
DESPLAZAMIENTO = 45.0               # mm por corrida, múltiplo de 5 mm (vuelta del husillo)
REPOSO_S = 1.0                      # quieto antes y después
FS_INTERF = {1.0: 50e3, 2.0: 50e3, 5.0: 100e3}   # S/s de la placa/osciloscopio (supuesto)
FS_ENC = 1000.0                     # telemetría del RP2040 (medido)

# Lazo cerrado: segundo orden equivalente (supuesto; no tenemos el Bode del lazo).
FN_LAZO_HZ, ZETA_LAZO = 40.0, 0.7

# Cogging (fuerza): 1,46 µm de error a 2 mm/s sin anti-cogging, 0,06 µm con él
# (medido, CAMBIOS_CALIBRACION_2026-10-20). La compensación se desvanece de 2 a 3 mm/s (medido).
COG_SIN_COMP_UM = 1.46
COG_RESIDUO = 0.06 / 1.46
COG_2X_FUERZA = 0.3                 # 2.º armónico de la fuerza (supuesto)

# No linealidad del encoder (supuesto: la cifra de 4,4–5,2 µm RMS sobreestima).
ENC_1X_UM, ENC_2X_UM = 1.0, 0.4
# Husillo: error periódico por vuelta y error de escala (supuestos).
HUS_1X_UM, HUS_2X_UM, HUS_ESCALA_PPM = 1.5, 0.4, 40.0
# Oscilación del lazo a ~214 Hz con VEL_KP 0,12: unos 1,5 nm (medido en reposo, 06/11).
LAZO_HZ, LAZO_NM = 214.0, 1.5
# Modo mecánico que solo ve el carro (supuesto, para probar el análisis).
MEC_HZ, MEC_NM = 120.0, 4.0
# Ruido
ENC_RUIDO_NM = 0.8                  # muestra a muestra (medido, 06/11)
AIRE_RMS_NM = 3.0                   # turbulencia/deriva del camino óptico, 1/f (supuesto)
DET_RUIDO = 0.003                   # ruido del detector, relativo a la amplitud de franja
ADC_BITS, ADC_RANGO_V = 12, 2.5
# Imperfecciones del Michelson en cuadratura (supuestas)
OFFSET_1, OFFSET_2, GANANCIA_2, ALFA_GRAD = 1.25, 1.18, 0.85, 7.0
VISIB_CAIDA = 0.25                  # la amplitud de franja cae 25 % a lo largo del recorrido
# Relojes
RELOJ_PPM, RELOJ_DESFASE_S = 30.0, 0.137

# Paleta (dataviz, tema claro)
C_I, C_E, C_3 = "#2a78d6", "#eb6834", "#1baf7a"
TINTA, TINTA2, GRILLA, FONDO = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "figure.facecolor": FONDO, "axes.facecolor": FONDO, "axes.edgecolor": TINTA2,
    "axes.labelcolor": TINTA, "xtick.color": TINTA2, "ytick.color": TINTA2,
    "axes.grid": True, "grid.color": GRILLA, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
    "legend.frameon": False, "lines.linewidth": 1.0,
})


# ======================================================================
# Modelo físico
# ======================================================================
def T_lazo(f):
    s, wn = 2j * np.pi * np.asarray(f, float), 2 * np.pi * FN_LAZO_HZ
    return wn**2 / (s**2 + 2 * ZETA_LAZO * wn * s + wn**2)


def S_lazo(f):
    return 1 - T_lazo(f)


def G_fuerza(f):
    """Fuerza -> posición en lazo cerrado, P·S con P = 1/(m s²), salvo la masa."""
    return S_lazo(f) / (2 * np.pi * np.asarray(f, float)) ** 2


def comp_cogging(v):
    frac = np.clip(3.0 - v, 0.0, 1.0)            # 1 hasta 2 mm/s, 0 desde 3 mm/s
    return 1.0 - frac * (1.0 - COG_RESIDUO)


def componentes_verdad(v):
    """Lista de componentes senoidales: (nombre, tipo, período_mm o f_Hz, A_i_nm, A_e_nm, fase_i, fase_e)."""
    lp = 5.0 / 200
    f1 = v / lp
    ref = abs(G_fuerza(2.0 / lp))                # 80 Hz: donde se midieron los 1,46 µm
    k = comp_cogging(v)
    cog1 = COG_SIN_COMP_UM * 1e3 * abs(G_fuerza(f1)) / ref * k
    cog2 = COG_SIN_COMP_UM * 1e3 * COG_2X_FUERZA * abs(G_fuerza(2 * f1)) / ref * k
    out = []
    ph = RNG.uniform(0, 2 * np.pi, 10)
    # Fuerza: el carro y el motor se mueven igual (husillo rígido a estas frecuencias).
    out.append(("paso motor 1x (200/vuelta)", "esp", lp, cog1, cog1, ph[0], ph[0]))
    out.append(("paso motor 2x", "esp", lp / 2, cog2, cog2, ph[1], ph[1]))
    # Encoder: x_i = −n·T, x_e = n·S.
    for nombre, per, A, p in (("encoder 1x (30/vuelta)", 5 / 30, ENC_1X_UM, ph[2]),
                              ("encoder 2x = imanes (60/vuelta)", 5 / 60, ENC_2X_UM, ph[3])):
        f = v / per
        T, S = T_lazo(f), S_lazo(f)
        out.append((nombre, "esp", per, A * 1e3 * abs(T), A * 1e3 * abs(S),
                    p + np.pi + np.angle(T), p + np.angle(S)))
    # Husillo: solo el carro.
    out.append(("husillo 1x", "esp", 5.0, HUS_1X_UM * 1e3, 0.0, ph[4], 0.0))
    out.append(("husillo 2x", "esp", 2.5, HUS_2X_UM * 1e3, 0.0, ph[5], 0.0))
    # Temporales
    out.append(("oscilación del lazo (~214 Hz)", "tem", LAZO_HZ, LAZO_NM, LAZO_NM, ph[6], ph[6]))
    out.append(("modo mecánico 120 Hz (solo carro)", "tem", MEC_HZ, MEC_NM, 0.0, ph[7], 0.0))
    return out


def perfil(t, v):
    """Trapecio: reposo, rampa, crucero de L_CRUCERO mm, rampa, reposo."""
    ta = v / ACEL
    da = 0.5 * ACEL * ta**2
    L_CRUCERO = DESPLAZAMIENTO - 2 * da
    tc = L_CRUCERO / v
    t1, t2, t3, t4 = REPOSO_S, REPOSO_S + ta, REPOSO_S + ta + tc, REPOSO_S + 2 * ta + tc
    x = np.zeros_like(t)
    m = (t >= t1) & (t < t2); x[m] = 0.5 * ACEL * (t[m] - t1) ** 2
    m = (t >= t2) & (t < t3); x[m] = da + v * (t[m] - t2)
    m = (t >= t3) & (t < t4); x[m] = da + L_CRUCERO + v * (t[m] - t3) - 0.5 * ACEL * (t[m] - t3) ** 2
    x[t >= t4] = 2 * da + L_CRUCERO
    return x, t4 + REPOSO_S, (t2, t3)


def sumar(t, x_mm, comps, sensor):
    """Suma de componentes en nm. Las espaciales dependen de x; las temporales de t.
    La amplitud de la oscilación del lazo varía lentamente, como en las mediciones."""
    y = np.zeros_like(t)
    for nombre, tipo, p, Ai, Ae, fi, fe in comps:
        A, fase = (Ai, fi) if sensor == "i" else (Ae, fe)
        if A == 0:
            continue
        if tipo == "esp":
            y += A * np.sin(2 * np.pi * x_mm / p + fase)
        else:
            am = 1 + 0.3 * np.sin(2 * np.pi * 0.05 * t) if "lazo" in nombre else 1.0
            y += A * am * np.sin(2 * np.pi * p * t + fase)
    return y


def ruido_1f(n, fs, rms, f_min=0.05):
    X = np.fft.rfft(RNG.standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / fs)
    X /= np.sqrt(np.maximum(f, f_min))
    X[0] = 0
    y = np.fft.irfft(X, n)
    return y / y.std() * rms


def simular(v, fs, modo="cuadratura"):
    comps = componentes_verdad(v)
    # --- Michelson
    T_tot = perfil(np.array([0.0]), v)[1]
    t = np.arange(0, T_tot, 1 / fs)
    x_nom, _, (tc0, tc1) = perfil(t, v)
    t_aire = np.arange(0, T_tot + 1, 1e-3)
    aire = np.interp(t, t_aire, ruido_1f(len(t_aire), 1e3, AIRE_RMS_NM))
    x_i = x_nom * (1 + HUS_ESCALA_PPM * 1e-6) + (sumar(t, x_nom, comps, "i") + aire) * 1e-6
    fase = 4 * np.pi * an.N_AIRE * x_i * 1e6 / an.LAMBDA_NM
    amp = 1.0 - VISIB_CAIDA * x_nom / x_nom.max()
    a = 1.0 * amp
    I1 = OFFSET_1 + a * np.cos(fase) + DET_RUIDO * RNG.standard_normal(len(t))
    I2 = OFFSET_2 + GANANCIA_2 * a * np.sin(fase + np.radians(ALFA_GRAD)) + DET_RUIDO * RNG.standard_normal(len(t))
    lsb = ADC_RANGO_V / 2**ADC_BITS
    I1, I2 = np.round(I1 / lsb) * lsb, np.round(I2 / lsb) * lsb
    # --- Encoder: su propio reloj, arranca 0,137 s después y corre 30 ppm rápido.
    k = np.arange(int((T_tot - RELOJ_DESFASE_S - 0.01) * FS_ENC))
    t_e_verdad = RELOJ_DESFASE_S + k / (FS_ENC * (1 + RELOJ_PPM * 1e-6))
    xe_nom = perfil(t_e_verdad, v)[0]
    x_e = xe_nom + (sumar(t_e_verdad, xe_nom, comps, "e") + ENC_RUIDO_NM * RNG.standard_normal(len(k))) * 1e-6
    t_e = k / FS_ENC
    return dict(v=v, fs=fs, t=t, I1=I1, I2=I2, x_i_verdad=x_i, t_e=t_e, x_e=x_e,
                comps=comps, crucero=(tc0, tc1))


def demodular(sim, modo):
    if modo == "cuadratura":
        fase, e = an.fase_cuadratura(sim["I1"], sim["I2"])
        return sim["t"], an.fase_a_mm(fase), e
    # Un detector: solo el tramo en movimiento (en reposo no hay franja que seguir).
    tc0, tc1 = sim["crucero"]
    m = (sim["t"] > tc0 - 0.1) & (sim["t"] < tc1 + 0.1)
    fase = an.fase_un_detector(sim["I1"][m], sim["fs"], 0.2 * an.frecuencia_franja_hz(sim["v"]))
    return sim["t"][m], an.fase_a_mm(fase), None


# ======================================================================
# Corridas
# ======================================================================
def correr():
    SALIDA.mkdir(exist_ok=True)
    res, sims, elipses = {}, {}, {}
    for v in VELOCIDADES:
        print(f"Simulando {v} mm/s ...", flush=True)
        s = simular(v, FS_INTERF[v])
        t, x, e = demodular(s, "cuadratura")
        r = an.analizar_corrida(t, x, s["t_e"], s["x_e"])
        r["escala_ppm"] = an.escala_entre_reposos(t, x, s["t_e"], s["x_e"])
        r["error_demod_nm"] = float(np.std((x - s["x_i_verdad"]) * 1e6))
        res[v], elipses[v] = r, e
        sims[v] = dict(comps=s["comps"])
        if v == 2.0:
            sims["v2_full"] = s
    # Caso B: un detector a 2 mm/s, mismos datos
    s2 = sims.pop("v2_full")
    tB, xB, _ = demodular(s2, "un_detector")
    resB = an.analizar_corrida(tB, xB, s2["t_e"], s2["x_e"])
    # e_m con distintos grados de sincronismo (2 mm/s)
    tA, xA, _ = demodular(s2, "cuadratura")
    em = {
        "exacto (disparo por hardware, deriva corregida)": an.espectro_em(tA, xA, s2["t_e"], s2["x_e"], RELOJ_DESFASE_S, RELOJ_PPM),
        "un pulso al inicio, sin corregir 30 ppm": an.espectro_em(tA, xA, s2["t_e"], s2["x_e"], RELOJ_DESFASE_S, 0.0),
        "alineado a mano, 5 ms de error": an.espectro_em(tA, xA, s2["t_e"], s2["x_e"], RELOJ_DESFASE_S + 5e-3, RELOJ_PPM),
    }
    # Caso C: 5 mm/s a 25 kS/s
    sC = simular(5.0, 25e3)
    tC, xC, _ = demodular(sC, "cuadratura")
    vC = np.polyfit(tC[(tC > sC["crucero"][0]) & (tC < sC["crucero"][1])],
                    xC[(tC > sC["crucero"][0]) & (tC < sC["crucero"][1])], 1)[0]
    casoC = dict(t=tC, x=xC, x_verdad=sC["x_i_verdad"], v_aparente=vC)
    return res, sims, elipses, resB, casoC, s2, em


# ======================================================================
# Figuras
# ======================================================================
def figuras(res, sims, elipses, resB, casoC, s2, em):
    # 1. Señal cruda y Lissajous (2 mm/s)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(10, 3.4), gridspec_kw=dict(width_ratios=[2.2, 1]))
    i0 = np.searchsorted(s2["t"], 10.0)
    sel = slice(i0, i0 + int(3e-3 * s2["fs"]))
    tt = (s2["t"][sel] - s2["t"][i0]) * 1e3
    a1.plot(tt, s2["I1"][sel], color=C_I, label="I1 (coseno)")
    a1.plot(tt, s2["I2"][sel], color=C_E, label="I2 (seno, 7° de error)")
    a1.set_title("Fotodetectores a 2 mm/s: 6,3 kHz de franja (3 ms)", loc="left")
    a1.set_xlabel("tiempo (ms)"); a1.set_ylabel("tensión (V)"); a1.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), ncol=2)
    st = slice(None, None, 97)
    a2.plot(s2["I1"][st], s2["I2"][st], ".", ms=0.6, color=TINTA2, alpha=0.4)
    e = elipses[2.0]
    ph = np.linspace(0, 2 * np.pi, 400)
    a2.plot(e["x0"] + e["a"] * np.cos(ph), e["y0"] + e["b"] * np.sin(ph + e["alfa"]), color=C_I, lw=1.5)
    a2.set_aspect("equal"); a2.set_title("Lissajous y elipse ajustada", loc="left")
    a2.set_xlabel("I1 (V)"); a2.set_ylabel("I2 (V)")
    fig.tight_layout(); fig.savefig(SALIDA / "fig1_senal_cruda.png", dpi=150); plt.close(fig)

    # 2. Residuos a 2 mm/s contra posición
    r = res[2.0]
    fig, axs = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    for ax, k, c, nom in ((axs[0], "interf", C_I, "x_i (interferómetro)"), (axs[1], "encoder", C_E, "x_e (encoder)")):
        d = r[k]
        xp = (d["x"] - d["x"][0])
        ax.plot(xp, d["delta"], color=c, lw=0.6)
        ax.set_ylabel("residuo (nm)"); ax.set_title(f"{nom}: x − recta, 2 mm/s", loc="left")
    axs[1].set_xlabel("posición en la ventana (mm)")
    caja = dict(boxstyle="round,pad=0.3", fc=FONDO, ec="none", alpha=0.9)
    axs[0].annotate("ondulación de 5 mm: husillo", xy=(0.01, 0.04), xycoords="axes fraction", color=TINTA2, bbox=caja)
    axs[1].annotate("el husillo no aparece: el encoder está del lado del motor", xy=(0.01, 0.04),
                    xycoords="axes fraction", color=TINTA2, bbox=caja)
    fig.tight_layout(); fig.savefig(SALIDA / "fig2_residuos_2mms.png", dpi=150); plt.close(fig)

    # 3. Espectros x_i y x_e por velocidad
    fig, axs = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for ax, v in zip(axs, VELOCIDADES):
        r = res[v]
        for k, c, nom in (("interf", C_I, "x_i (interferómetro)"), ("encoder", C_E, "x_e (encoder)")):
            d = r[k]
            ax.loglog(d["f"][1:], np.maximum(d["amp"][1:], 1e-3), color=c, lw=0.7, label=nom)
        for cc in r["componentes"]:
            a = np.nanmax([cc["A_i"], cc["A_e"]])
            if cc["sensores"] in ("no detectado", "fuera de banda") or np.isnan(a):
                continue
            ax.annotate(cc["nombre"].split(" (")[0], xy=(cc["f"], a), xytext=(0, 8),
                        textcoords="offset points", ha="center", fontsize=7, color=TINTA2)
        for p in r["desconocidos_i"][:3]:
            ax.annotate(f"? {p['f']:.0f} Hz", xy=(p["f"], p["amp"]), xytext=(4, -3),
                        textcoords="offset points", ha="left", va="top", fontsize=7, color=TINTA)
        ax.set_ylim(1e-2, 5e3); ax.set_xlim(0.1, 2500)
        ax.set_ylabel("amplitud (nm)"); ax.set_title(f"{v:g} mm/s", loc="left")
    axs[0].legend(loc="upper right")
    axs[-1].set_xlabel("frecuencia (Hz)")
    fig.tight_layout(); fig.savefig(SALIDA / "fig3_espectros.png", dpi=150); plt.close(fig)

    # 4. Espectro espacial de x_i: las tres velocidades en ciclos/mm
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for v, c in zip(VELOCIDADES, (C_I, C_E, C_3)):
        d = res[v]["interf"]
        ax.loglog(d["f"][1:] / v, np.maximum(d["amp"][1:], 1e-3), color=c, lw=0.7, label=f"{v:g} mm/s")
    for nombre, per in an.PERIODOS_ESPACIALES_MM.items():
        ax.axvline(1 / per, color=GRILLA, lw=1, zorder=0)
        ax.annotate(nombre.split(" (")[0], xy=(1 / per, 2.5e3), rotation=90, fontsize=7, color=TINTA2,
                    ha="right", va="top")
    for v, c in zip(VELOCIDADES, (C_I, C_E, C_3)):
        for f0 in (LAZO_HZ, MEC_HZ):
            ax.annotate("", xy=(f0 / v, 0.03), xytext=(f0 / v, 0.012),
                        arrowprops=dict(arrowstyle="-|>", color=c, lw=1))
    ax.set_xlim(0.1, 300); ax.set_ylim(1e-2, 5e3)
    ax.set_xlabel("frecuencia espacial (ciclos/mm)"); ax.set_ylabel("amplitud (nm)")
    ax.set_title("x_i en función de la posición: lo espacial coincide, lo temporal (flechas: 120 y 214 Hz) se corre",
                 loc="left")
    ax.legend(loc="upper right")
    fig.tight_layout(); fig.savefig(SALIDA / "fig4_espectro_espacial.png", dpi=150); plt.close(fig)

    # 6. Submuestreo
    fig, ax = plt.subplots(figsize=(10, 3.4))
    st = slice(None, None, 25)
    t = casoC["t"]
    tv = np.arange(len(casoC["x_verdad"])) / 25e3
    ax.plot(tv[st], casoC["x_verdad"][st], color=C_3, lw=1.5, label="posición real")
    ax.plot(t[st], casoC["x"][st], color=C_E, lw=1.5, label="demodulada con 25 kS/s")
    ax.set_xlabel("tiempo (s)"); ax.set_ylabel("posición (mm)")
    ax.set_title(f"5 mm/s (franja de 15,8 kHz) muestreado a 25 kS/s: la velocidad aparente es "
                 f"{casoC['v_aparente']:+.2f} mm/s", loc="left")
    ax.legend(loc="center left")
    fig.tight_layout(); fig.savefig(SALIDA / "fig6_submuestreo.png", dpi=150); plt.close(fig)

    # 5. e_m = x_e − x_i según el sincronismo
    fig, ax = plt.subplots(figsize=(10, 3.8))
    for (nom, d), c in reversed(list(zip(em.items(), (C_I, C_E, C_3)))):
        ax.loglog(d["f"][1:], np.maximum(d["amp"][1:], 1e-3), color=c, lw=0.7, label=nom)
    for nombre, f0 in (("encoder 1x", 12), ("imanes", 24), ("paso motor", 80), ("husillo", 0.4)):
        ax.annotate(nombre, xy=(f0, 2e3), ha="center", fontsize=7, color=TINTA2)
    ax.set_xlim(0.1, 500); ax.set_ylim(1e-2, 5e3)
    ax.set_xlabel("frecuencia (Hz)"); ax.set_ylabel("amplitud (nm)")
    ax.set_title("e_m = x_e − x_i a 2 mm/s: con buen sincronismo el paso del motor (fuerza) se cancela", loc="left")
    h, l = ax.get_legend_handles_labels()
    ax.legend(h[::-1], l[::-1], loc="upper right", bbox_to_anchor=(1.0, 0.88))
    fig.tight_layout(); fig.savefig(SALIDA / "fig5_em_sincronismo.png", dpi=150); plt.close(fig)


# ======================================================================
# Informe
# ======================================================================
def verdad_por_nombre(comps):
    return {c[0]: (c[3], c[4]) for c in comps}


def tabla_validacion(res, sims):
    lin = ["| v (mm/s) | Componente | f (Hz) | x_i puesto | x_i medido | x_e puesto | x_e medido | Sensores |",
           "|---|---|---|---|---|---|---|---|"]
    for v in VELOCIDADES:
        ver = verdad_por_nombre(sims[v]["comps"])
        for c in res[v]["componentes"]:
            if c["nombre"] not in ver:
                continue
            Ai, Ae = ver[c["nombre"]]
            f = lambda a: "—" if np.isnan(a) else f"{a:.1f}"  # noqa: E731
            lin.append(f"| {v:g} | {c['nombre']} | {c['f']:.1f} | {Ai:.1f} | {f(c['A_i'])} | "
                       f"{Ae:.1f} | {f(c['A_e'])} | {c['sensores']} |")
    return "\n".join(lin)


def informe(res, sims, elipses, resB, casoC, em):
    picos = {v: [p["f"] for p in res[v]["desconocidos_i"][:5]] for v in VELOCIDADES}
    cruz = an.clasificar_entre_velocidades(picos)
    lin = []
    lin.append("## Picos no esperados en x_i y su clasificación entre velocidades\n")
    lin.append("| v (mm/s) | f (Hz) | Amplitud (nm) | Tipo | λ (mm) |\n|---|---|---|---|---|")
    for v in VELOCIDADES:
        for p in res[v]["desconocidos_i"][:5]:
            tipo = next((c[2] for c in cruz if c[0] == v and c[1] == p["f"]), "")
            lam = next((c[3] for c in cruz if c[0] == v and c[1] == p["f"]), None)
            lin.append(f"| {v:g} | {p['f']:.1f} | {p['amp']:.1f} | {tipo} | {'' if lam is None else f'{lam:.4f}'} |")
    picos_tabla = "\n".join(lin)

    resumen = ["| v (mm/s) | Ventana | Ripple de v RMS | Ripple de v pico a pico | Escala x_i/x_e | Error de demodulación |",
               "|---|---|---|---|---|---|"]
    for v in VELOCIDADES:
        r = res[v]
        resumen.append(f"| {v:g} | {r['interf']['L']:.1f} mm / {r['interf']['T']:.1f} s | "
                       f"{r['ripple_rms_um_s']:.2f} µm/s ({r['ripple_rms_um_s'] / v / 10:.2f} %) | "
                       f"{r['ripple_pp_pct']:.1f} % | {r['escala_ppm']:+.1f} ppm | {r['error_demod_nm']:.2f} nm RMS |")

    compB = ["| Componente | Cuadratura (nm) | Un detector (nm) |", "|---|---|---|"]
    for ca, cb in zip(res[2.0]["componentes"], resB["componentes"]):
        if ca["sensores"] in ("no detectado", "fuera de banda"):
            continue
        compB.append(f"| {ca['nombre']} | {ca['A_i']:.1f} | {cb['A_i']:.1f} |")

    req = ["| v (mm/s) | Frecuencia de franja | Muestreo mínimo (2,5×) | Recomendado (5×) | Muestras por canal, 255 mm |",
           "|---|---|---|---|---|"]
    for v in (0.5, 1, 2, 3, 5):
        ff = an.frecuencia_franja_hz(v)
        n = 5 * ff * (255 / v + 2 * v / ACEL + 2)
        req.append(f"| {v:g} | {ff / 1e3:.2f} kHz | {2.5 * ff / 1e3:.0f} kS/s | {5 * ff / 1e3:.0f} kS/s | {n / 1e6:.0f} M |")

    tem = ["| Sincronismo | husillo 1x (0,4 Hz) | encoder 1x (12 Hz) | imanes (24 Hz) | paso motor (80 Hz) |",
           "|---|---|---|---|---|"]
    for nom, d in em.items():
        vals = [an.amplitud_en(d["f"], d["amp"], f0, 3 * d["df"], 6 * d["df"])[0] for f0 in (0.4, 12, 24, 80)]
        tem.append(f"| {nom} | " + " | ".join(f"{a:.1f}" for a in vals) + " |")
    tem.append(f"| puesto (n del encoder, husillo; fuerza = 0) | {HUS_1X_UM * 1e3:.0f} | {ENC_1X_UM * 1e3:.0f} | "
               f"{ENC_2X_UM * 1e3:.0f} | 0 |")
    e = elipses[2.0]
    return {
        "EM": "\n".join(tem),
        "VALIDACION": tabla_validacion(res, sims),
        "TABLA_V1": an.tabla_componentes(res[1.0]),
        "TABLA_V2": an.tabla_componentes(res[2.0]),
        "TABLA_V5": an.tabla_componentes(res[5.0]),
        "PICOS": picos_tabla,
        "RESUMEN": "\n".join(resumen),
        "UN_DETECTOR": "\n".join(compB),
        "REQ": "\n".join(req),
        "ELIPSE": (f"offsets {e['x0']:.3f} / {e['y0']:.3f} V (puestos {OFFSET_1} / {OFFSET_2}), "
                   f"relación de ganancias {e['b'] / e['a']:.3f} (puesta {GANANCIA_2}), "
                   f"error de fase {np.degrees(e['alfa']):.2f}° (puesto {ALFA_GRAD}°)"),
        "V_C": f"{casoC['v_aparente']:+.3f}",
    }


if __name__ == "__main__":
    res, sims, elipses, resB, casoC, s2, em = correr()
    figuras(res, sims, elipses, resB, casoC, s2, em)
    partes = informe(res, sims, elipses, resB, casoC, em)
    for k, val in partes.items():
        print(f"\n=== {k} ===\n{val}")
