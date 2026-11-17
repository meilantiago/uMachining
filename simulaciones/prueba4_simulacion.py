#!/usr/bin/env python3
"""Simulación de la prueba 4 (respuesta en frecuencia) con un Michelson.

Se simula el lazo del firmware tal como está (PI de posición -> PI de velocidad
-> adelanto del campo, filtro de velocidad de 4 ms, perfil a 2 kHz, servo a
16 kHz) sobre un modelo mecánico de motor, acople, husillo, tuerca y carro con
fricción de Dahl. Se comanda la senoide a una frecuencia por vez y se registran:
  - la telemetría del RP2040 (x_c, x_e) a 1 kS/s con su reloj;
  - las dos señales crudas del Michelson con el reloj del INTI.
Después se aplica tools/prueba4_analisis.py y se compara con la verdad.

Corridas:
  A. 30 mm, A = 1 µm                       <- caso principal
  B. 30 mm, A = 100 nm                      (linealidad: fricción casi sin deslizar)
  C. 230 mm, A = 1 µm                       (husillo más largo: más blando)
  D. 30 mm, A = 1 µm sobre rampa de 2 mm/s (fricción deslizando)
  E. 230 mm, igual que D pero bajando (husillo largo, deslizando)

Salida: simulaciones/prueba4/ (figuras y RESULTADOS.md).
"""
from __future__ import annotations

import math
import pathlib
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "tools"))
import prueba2_analisis as p2  # noqa: E402
import prueba4_analisis as an  # noqa: E402

SALIDA = pathlib.Path(__file__).resolve().parent / "prueba4"
RNG = np.random.default_rng(20261118)

# ======================================================================
# Firmware (src/hw_config.h, servo_controller.cpp): medido
# ======================================================================
POS_KP, POS_KI, POS_OUT, POS_WINDUP = 60.0, 100.0, 2 * math.pi, 0.5 * math.pi
VEL_KP, VEL_KI, VEL_LIM = 0.12, 54.0, 0.45 * math.pi
VEL_TC = 0.004
FS_SERVO, FS_PERFIL, FS_TELEM = 16000, 2000, 1000
PASO_MM = 5.0
R_MM = PASO_MM / (2 * math.pi)          # mm por radián de motor
R_M = R_MM * 1e-3

# ======================================================================
# Mecánica (supuesta salvo donde se indica)
# ======================================================================
J1 = 6e-6        # rotor + rueda de imanes [kg m²]
J2 = 1e-6        # husillo + medio acople
M_CARRO = 1.0    # kg
K_ACOPLE, Z_ACOPLE = 30.0, 0.05          # N·m/rad
T0 = 0.12        # torque por radián de adelanto (82–165 mN·m según tensión; PLAN_DE_PRUEBAS_MODULAR)
TAU_E = 1.12e-3  # L/R = 37 mH / 33 Ω (medido)
B_EMF = 6.4e-3   # Kt²/R = 0,46²/33 (medido): amortiguamiento por fuerza contraelectromotriz
B_VISC = 3.6e-3  # viscoso de husillo y tuerca
F_COUL = 0.020   # N·m, rozamiento 14–30 mN·m (medido)
SIGMA0 = 30.0    # N·m/rad: rigidez de la fricción antes de deslizar (~0,5 µm de carro)
EA_HUSILLO = 200e9 * math.pi * 0.003**2  # acero, diámetro de raíz 6 mm
K_TUERCA = 30e6  # N/m
L0_RODAMIENTO = 0.040                    # m entre el rodamiento fijo y la posición 0
Z_AXIAL = 0.03


def k_axial(x0_mm):
    k_h = EA_HUSILLO / (L0_RODAMIENTO + x0_mm * 1e-3)
    return 1 / (1 / k_h + 1 / K_TUERCA)


# Encoder (medido / supuesto como en la prueba 2)
ENC_1X_UM, ENC_2X_UM, ENC_RUIDO_NM = 1.0, 0.4, 0.8
# Michelson
FS_INTERF = 25e3
AIRE_RMS_NM = 3.0
RELOJ_PPM, RELOJ_DESFASE_S = 30.0, 0.137

# Barrido
FRECUENCIAS = [1, 2, 5, 10, 20, 30, 50, 70, 100, 130, 150, 170, 200]
T_RAMPA_ENV, T_ASENT = 0.2, 0.3
MARCA_UM, MARCA_SUBIDA, MARCA_ESPERA = 50.0, 0.2, 0.3   # 50 µm: el carro desliza

C1, C2, C3, C4 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100"
TINTA, TINTA2, GRILLA, FONDO = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
plt.rcParams.update({
    "figure.facecolor": FONDO, "axes.facecolor": FONDO, "axes.edgecolor": TINTA2,
    "axes.labelcolor": TINTA, "xtick.color": TINTA2, "ytick.color": TINTA2,
    "axes.grid": True, "grid.color": GRILLA, "grid.linewidth": 0.6,
    "axes.spines.top": False, "axes.spines.right": False, "font.size": 9,
    "legend.frameon": False, "lines.linewidth": 1.2,
})


# ======================================================================
# Perfil: marcas + senoides (reloj del RP2040)
# ======================================================================
def armar_perfil(A_mm, v_rampa=0.0):
    segs, partes = [], []
    t = 0.5
    marcas = [t]
    t += 2 * MARCA_SUBIDA + MARCA_ESPERA + 0.3
    for f in FRECUENCIAS:
        tm = math.ceil(max(5.0 / f, 0.5) * f) / f
        t0 = t + T_RAMPA_ENV + T_ASENT
        segs.append(dict(f=f, t0=t0, t1=t0 + tm, ini=t, fin=t0 + tm + T_RAMPA_ENV))
        t = t0 + tm + T_RAMPA_ENV
    t += 0.3
    marcas.append(t)
    t_fin = t + 2 * MARCA_SUBIDA + MARCA_ESPERA + 0.5

    tp = np.arange(0, t_fin, 1 / FS_PERFIL)
    x = np.zeros_like(tp)
    v = np.zeros_like(tp)
    for s in segs:
        m = (tp >= s["ini"]) & (tp < s["fin"])
        tt = tp[m] - s["ini"]
        dur = s["fin"] - s["ini"]
        env = np.ones_like(tt)
        a = tt < T_RAMPA_ENV
        env[a] = 0.5 - 0.5 * np.cos(np.pi * tt[a] / T_RAMPA_ENV)
        b = tt > dur - T_RAMPA_ENV
        env[b] = 0.5 - 0.5 * np.cos(np.pi * (dur - tt[b]) / T_RAMPA_ENV)
        w = 2 * np.pi * s["f"]
        x[m] += A_mm * env * np.sin(w * tp[m])
        v[m] += A_mm * env * w * np.cos(w * tp[m])     # la envolvente es lenta: se ignora su derivada
    for tm_ in marcas:
        h = MARCA_UM * 1e-3
        for t_a, sgn in ((tm_, 1), (tm_ + MARCA_SUBIDA + MARCA_ESPERA, -1)):
            m = (tp >= t_a) & (tp < t_a + MARCA_SUBIDA)
            u = (tp[m] - t_a) / MARCA_SUBIDA
            base = h if sgn < 0 else 0.0
            x[m] += base + sgn * h * (0.5 - 0.5 * np.cos(np.pi * u))
            v[m] += sgn * h * 0.5 * np.pi / MARCA_SUBIDA * np.sin(np.pi * u)
        x[(tp >= tm_ + MARCA_SUBIDA) & (tp < tm_ + MARCA_SUBIDA + MARCA_ESPERA)] += h
    x += v_rampa * tp
    v += v_rampa
    return tp, x, v, segs, marcas


# ======================================================================
# Lazo + mecánica (un paso de servo, 2 subpasos mecánicos)
# ======================================================================
def simular_dinamica(x0_mm, xc, vc, seed):
    rng = np.random.default_rng(seed)
    n_perfil = len(xc)
    n = n_perfil * (FS_SERVO // FS_PERFIL)
    dt = 1.0 / FS_SERVO
    h = dt / 2
    kax = k_axial(x0_mm)
    kaxt = kax * R_M * R_M
    cax = 2 * Z_AXIAL * math.sqrt(kax * M_CARRO) * R_M * R_M
    mt = M_CARRO * R_M * R_M
    jr = J1 * J2 / (J1 + J2)
    cc = 2 * Z_ACOPLE * math.sqrt(K_ACOPLE * jr)
    alfa = VEL_TC / (VEL_TC + dt)
    w1e, w2e = 2 * math.pi / (5.0 / 30), 2 * math.pi / (5.0 / 60)   # períodos del encoder y de los imanes
    a1, a2 = ENC_1X_UM * 1e-3, ENC_2X_UM * 1e-3
    f1, f2 = rng.uniform(0, 2 * math.pi, 2)
    ruido = (ENC_RUIDO_NM * 1e-6 * rng.standard_normal(n)).tolist()
    n0 = a1 * math.sin(w1e * x0_mm + f1) + a2 * math.sin(w2e * x0_mm + f2)

    # Estado (desvíos respecto de x0, en rad de motor; carro en rad equivalentes)
    th1 = w1 = th2 = w2 = thx = wx = 0.0
    tau = tf = 0.0
    ip = iv = 0.0
    e_prev = ev_prev = 0.0
    vf = 0.0
    th_e_prev = 0.0
    tau_cmd = 0.0
    lp = [0.0]
    xs = np.empty(n)
    encs = np.empty(n)
    mot = np.empty(n)
    xcs = np.empty(n)
    xc_l, vc_l = xc.tolist(), vc.tolist()
    sin = math.sin
    for k in range(n):
        j = k >> 3
        xc_k = xc_l[j]
        vc_k = vc_l[j]
        x_mot = th1 * R_MM
        xa = x0_mm + x_mot
        enc = x_mot + a1 * sin(w1e * xa + f1) + a2 * sin(w2e * xa + f2) - n0 + ruido[k]
        th_e = enc / R_MM
        # PI de posición (Tustin), mismo recorte que el firmware
        e = xc_k / R_MM - th_e
        ip += 0.5 * POS_KI * dt * (e + e_prev)
        ip = POS_WINDUP if ip > POS_WINDUP else (-POS_WINDUP if ip < -POS_WINDUP else ip)
        vt = POS_KP * e + ip
        vt = POS_OUT if vt > POS_OUT else (-POS_OUT if vt < -POS_OUT else vt)
        vt += vc_k / R_MM
        e_prev = e
        # velocidad medida y filtrada
        vf = alfa * vf + (1 - alfa) * (th_e - th_e_prev) / dt
        th_e_prev = th_e
        ev = vt - vf
        iv += 0.5 * VEL_KI * dt * (ev + ev_prev)
        iv = VEL_LIM if iv > VEL_LIM else (-VEL_LIM if iv < -VEL_LIM else iv)
        ev_prev = ev
        lead = VEL_KP * ev + iv
        lead = VEL_LIM if lead > VEL_LIM else (-VEL_LIM if lead < -VEL_LIM else lead)
        tau_aplicar = tau_cmd                # un ciclo de retardo de cálculo
        tau_cmd = T0 * sin(lead)
        for _ in range(2):
            tau += (tau_aplicar - tau) * h / TAU_E
            tc = K_ACOPLE * (th1 - th2) + cc * (w1 - w2)
            fax = kaxt * (th2 - thx) + cax * (w2 - wx)
            sg = 1.0 if w2 > 0 else (-1.0 if w2 < 0 else 0.0)
            tf += h * SIGMA0 * w2 * (1 - tf / F_COUL * sg)
            w1 += h * (tau - tc - B_EMF * w1) / J1
            w2 += h * (tc - fax - tf - B_VISC * w2) / J2
            wx += h * fax / mt
            th1 += h * w1
            th2 += h * w2
            thx += h * wx
        xs[k] = thx * R_MM
        encs[k] = enc
        mot[k] = x_mot
        xcs[k] = xc_k
    t = np.arange(n) / FS_SERVO
    return dict(t=t, x=xs, enc=encs, mot=mot, xc=xcs)


def ruido_1f(n, fs, rms):
    X = np.fft.rfft(RNG.standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / fs)
    X /= np.sqrt(np.maximum(f, 0.05))
    X[0] = 0
    y = np.fft.irfft(X, n)
    return y / y.std() * rms


def michelson(t_rp, x_mm, x0_mm):
    """Señales crudas en el reloj del interferómetro."""
    t_fin_i = an.a_reloj_interf(t_rp[-1], RELOJ_DESFASE_S, RELOJ_PPM)
    t_i = np.arange(0, t_fin_i, 1 / FS_INTERF)
    tau = an.a_reloj_rp(t_i, RELOJ_DESFASE_S, RELOJ_PPM)
    x = np.interp(tau, t_rp, x_mm, left=x_mm[0], right=x_mm[-1])
    x = x + ruido_1f(len(t_i), FS_INTERF, AIRE_RMS_NM) * 1e-6
    fase = 4 * np.pi * p2.N_AIRE * (x0_mm + x) * 1e6 / p2.LAMBDA_NM
    I1 = 1.25 + np.cos(fase) + 0.003 * RNG.standard_normal(len(t_i))
    I2 = 1.18 + 0.85 * np.sin(fase + np.radians(7)) + 0.003 * RNG.standard_normal(len(t_i))
    lsb = 2.5 / 4096
    return t_i, np.round(I1 / lsb) * lsb, np.round(I2 / lsb) * lsb, x


# ======================================================================
# Corridas
# ======================================================================
CORRIDAS = {
    "A": dict(x0=30.0, A=1e-3, rampa=0.0, nombre="30 mm, 1 µm"),
    "B": dict(x0=30.0, A=1e-4, rampa=0.0, nombre="30 mm, 100 nm"),
    "C": dict(x0=230.0, A=1e-3, rampa=0.0, nombre="230 mm, 1 µm"),
    "D": dict(x0=30.0, A=1e-3, rampa=2.0, nombre="30 mm, 1 µm sobre rampa de 2 mm/s"),
    "E": dict(x0=230.0, A=1e-3, rampa=-2.0, nombre="230 mm, 1 µm sobre rampa de −2 mm/s"),
}


def correr_una(clave, cfg, seed):
    tp, xc, vc, segs, marcas = armar_perfil(cfg["A"], cfg["rampa"])
    t0 = time.time()
    d = simular_dinamica(cfg["x0"], xc, vc, seed)
    print(f"  {clave}: {len(d['t'])} pasos de servo ({d['t'][-1]:.1f} s) en {time.time() - t0:.0f} s", flush=True)
    # Telemetría a 1 kS/s
    q = FS_SERVO // FS_TELEM
    tel = dict(t=d["t"][::q], xc=d["xc"][::q], xe=d["enc"][::q])
    # Michelson y demodulación
    t_i, I1, I2, x_verdad_i = michelson(d["t"], d["x"], cfg["x0"])
    fase, elipse = p2.fase_cuadratura(I1, I2)
    x_i = p2.fase_a_mm(fase)
    x_i -= x_i[0]
    t_id, x_id = p2.diezmar(t_i, x_i, 5000.0)
    verdad = verdad_barrido(segs, tel, d)
    return dict(cfg=cfg, segs=segs, marcas=marcas, tel=tel, t_i=t_id, x_i=x_id, verdad=verdad,
                d=d, I1=I1, I2=I2, t_i_full=t_i, x_i_full=x_i, x_verdad_i=x_verdad_i)


def verdad_barrido(segs, tel, d):
    """Referencia sin errores de medición: x_c tal como lo registra la telemetría
    (el perfil a 2 kHz retiene cada valor 0,5 ms: ese retardo es parte del sistema),
    el motor y el carro reales, todos con el reloj del RP2040."""
    filas = []
    for s in segs:
        se = (tel["t"] >= s["t0"]) & (tel["t"] < s["t1"])
        sd = (d["t"] >= s["t0"]) & (d["t"] < s["t1"])
        Xc = an.ajuste_senoidal(tel["t"][se], tel["xc"][se], s["f"])
        Xm = an.ajuste_senoidal(d["t"][sd], d["mot"][sd], s["f"])
        Xi = an.ajuste_senoidal(d["t"][sd], d["x"][sd], s["f"])
        filas.append(dict(f=s["f"], A_c_nm=abs(Xc[0]) * 1e6, A_i_nm=abs(Xi[0]) * 1e6, Ge=Xm[0] / Xc[0],
                          Gi=Xi[0] / Xc[0], Gm=Xi[0] / Xm[0],
                          thd_i=float(np.sqrt(np.sum(np.abs(Xi[1:]) ** 2)) / abs(Xi[0]))))
    return filas


def analizar(r, modo):
    tel = r["tel"]
    if modo == "hardware":
        des, ppm = RELOJ_DESFASE_S, RELOJ_PPM
    elif modo == "marcas":
        des, ppm = an.estimar_sincronismo(r["t_i"], r["x_i"], tel["t"], tel["xe"], r["marcas"])
    elif modo == "una_marca":
        des, ppm = an.estimar_sincronismo(r["t_i"], r["x_i"], tel["t"], tel["xe"], r["marcas"], una_sola=True)
    else:   # sin sincronismo: se supone que arrancaron juntos
        des, ppm = 0.0, 0.0
    filas = an.analizar_barrido(r["segs"], tel["t"], tel["xc"], tel["xe"], r["t_i"], r["x_i"], des, ppm)
    return filas, des, ppm


# ======================================================================
# Figuras e informe
# ======================================================================
def bode(ax_m, ax_f, filas, clave, color, etiqueta, marcador="o", ls="-"):
    f = [r["f"] for r in filas]
    ax_m.semilogx(f, [an.db(r[clave]) for r in filas], ls, color=color, marker=marcador, ms=4, label=etiqueta)
    ax_f.semilogx(f, [an.fase_grad(r[clave]) for r in filas], ls, color=color, marker=marcador, ms=4, label=etiqueta)


def figuras(res, sinc):
    A = res["A"]
    # 1. Bode del lazo: G_e y G_i (corrida A, sincronismo por hardware)
    fig, (am, af) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    fa = sinc["hardware"][0]
    bode(am, af, fa, "Ge", C2, "x_e / x_c (encoder)")
    bode(am, af, fa, "Gi", C1, "x_i / x_c (interferómetro)")
    bode(am, af, A["verdad"], "Gi", TINTA2, "x_i / x_c verdadero", marcador="", ls=":")
    am.axhline(-3, color=GRILLA, lw=1)
    am.set_ylabel("ganancia (dB)"); af.set_ylabel("fase (°)"); af.set_xlabel("frecuencia (Hz)")
    am.set_title("Bode del lazo cerrado, 30 mm, A = 1 µm, sincronismo por hardware", loc="left")
    am.legend(loc="lower left")
    fig.tight_layout(); fig.savefig(SALIDA / "fig1_bode_lazo.png", dpi=150); plt.close(fig)

    # 3. Transmisión mecánica G_m en 30 y 230 mm, con el ajuste de resonancia
    fig, (am, af) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for clave, c in (("D", C1), ("E", C3)):
        filas = res[clave]["hw"]
        g0, fr = an.ajustar_resonancia(filas)
        f = np.array([r["f"] for r in filas])
        am.semilogx(f, [abs(r["Gm"]) for r in filas], "o", color=c, ms=4,
                    label=f"{res[clave]['cfg']['nombre']}: f_r ajustada = {fr:.0f} Hz")
        ff = np.logspace(0, np.log10(250), 200)
        am.semilogx(ff, g0 / np.abs(1 - (ff / fr) ** 2), "-", color=c, lw=0.8)
        af.semilogx(f, [an.fase_grad(r["Gm"]) for r in filas], "o-", color=c, ms=4)
    am.set_ylabel("|x_i / x_e|"); af.set_ylabel("fase (°)"); af.set_xlabel("frecuencia (Hz)")
    am.set_title("Transmisión motor → carro deslizando: la resonancia queda arriba de 200 Hz, pero se ve la curvatura",
                 loc="left")
    am.legend(loc="upper left")
    fig.tight_layout(); fig.savefig(SALIDA / "fig3_transmision.png", dpi=150); plt.close(fig)

    # 2. Dependencia de la amplitud y del régimen de fricción
    fig, (am, af) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for clave, c in (("A", C1), ("B", C2), ("D", C3)):
        bode(am, af, res[clave]["hw"], "Gi", c, res[clave]["cfg"]["nombre"])
    am.set_ylabel("ganancia (dB)"); af.set_ylabel("fase (°)"); af.set_xlabel("frecuencia (Hz)")
    am.set_title("x_i / x_c según amplitud y régimen de fricción", loc="left")
    am.legend(loc="lower left")
    fig.tight_layout(); fig.savefig(SALIDA / "fig2_amplitud_friccion.png", dpi=150); plt.close(fig)

    # 4. Error de fase según el sincronismo
    fig, ax = plt.subplots(figsize=(9, 3.6))
    f = [r["f"] for r in A["verdad"]]
    for (modo, (filas, _, _)), c in zip(sinc.items(), (C1, C3, C2, C4)):
        err = [an.fase_grad(r["Gi"] / v["Gi"]) for r, v in zip(filas, A["verdad"])]
        ax.semilogx(f, err, "o-", color=c, ms=4, label=modo.replace("_", " "))
    ax.set_ylim(-180, 180); ax.set_yticks([-180, -90, 0, 90, 180])
    ax.set_xlabel("frecuencia (Hz)"); ax.set_ylabel("error de fase de x_i/x_c (°)")
    ax.set_title("Error de fase según cómo se alinean los relojes (30 ppm de deriva)", loc="left")
    ax.legend(loc="lower left", ncol=4)
    fig.tight_layout(); fig.savefig(SALIDA / "fig4_sincronismo.png", dpi=150); plt.close(fig)

    # 5. Un detector contra cuadratura en un segmento de 50 Hz
    s = next(s for s in A["segs"] if s["f"] == 50)
    ti = A["t_i_full"]
    ti0 = an.a_reloj_interf(s["t0"], RELOJ_DESFASE_S, RELOJ_PPM)
    m = (ti > ti0) & (ti < ti0 + 0.06)
    x1 = p2.fase_a_mm(p2.fase_un_detector(A["I1"][m], FS_INTERF, 50.0))
    xq = A["x_i_full"][m]
    xv = A["x_verdad_i"][m]
    tt = (ti[m] - ti[m][0]) * 1e3
    fig, ax = plt.subplots(figsize=(9, 3.4))
    ax.plot(tt, (xv - xv.mean()) * 1e6, color=C3, lw=2.2, label="carro real")
    ax.plot(tt, (xq - xq.mean()) * 1e6, color=C1, lw=1.0, label="cuadratura")
    ax.plot(tt, (x1 - x1.mean()) * 1e6, color=C2, lw=1.0, label="un detector")
    ax.set_xlabel("tiempo (ms)"); ax.set_ylabel("posición (nm)")
    ax.set_title("50 Hz, 1 µm comandado: el carro cambia de sentido 100 veces por segundo y un detector no lo sigue",
                 loc="left")
    ax.legend(loc="upper right", ncol=3)
    fig.tight_layout(); fig.savefig(SALIDA / "fig5_un_detector.png", dpi=150); plt.close(fig)
    return float(np.std((x1 - x1.mean()) - (xv - xv.mean())) * 1e6), float(np.std((xq - xq.mean()) - (xv - xv.mean())) * 1e6)


def modo_transmision(x0_mm):
    """Primer modo del husillo + carro con el motor quieto (lo que ve x_i/x_e)."""
    kaxt = k_axial(x0_mm) * R_M * R_M
    K = np.array([[K_ACOPLE + kaxt, -kaxt], [-kaxt, kaxt]])
    M = np.diag([J2, M_CARRO * R_M * R_M])
    w2 = np.linalg.eigvals(np.linalg.solve(M, K)).real
    return float(np.sqrt(w2.min()) / 2 / np.pi)


def informe(res, sinc, err_1det):
    out = {}
    for clave in res:
        out[f"TABLA_{clave}"] = an.tabla(res[clave]["hw"])
        out[f"VERDAD_{clave}"] = an.tabla(res[clave]["verdad"])
    lin = ["| Sincronismo | Desfase estimado (ms) | Deriva estimada (ppm) | Error de fase a 20 Hz | a 100 Hz | a 200 Hz |",
           "|---|---|---|---|---|---|"]
    for modo, (filas, des, ppm) in sinc.items():
        errs = {r["f"]: an.fase_grad(r["Gi"] / v["Gi"]) for r, v in zip(filas, res["A"]["verdad"])}
        lin.append(f"| {modo} | {des * 1e3:.3f} | {ppm:.1f} | {errs[20]:+.2f}° | {errs[100]:+.2f}° | {errs[200]:+.2f}° |")
    lin.append(f"| real | {RELOJ_DESFASE_S * 1e3:.3f} | {RELOJ_PPM:.1f} | | | |")
    out["SINC"] = "\n".join(lin)
    lin = ["| Corrida | f_r ajustada (Hz) | k axial puesta (N/µm) | Modo real de la transmisión (Hz) | Ancho de banda x_i/x_c (Hz) |",
           "|---|---|---|---|---|"]
    for clave in res:
        g0, fr = an.ajustar_resonancia(res[clave]["hw"])
        k = k_axial(res[clave]["cfg"]["x0"])
        bw = an.ancho_de_banda(res[clave]["hw"])
        lin.append(f"| {res[clave]['cfg']['nombre']} | {fr:.0f} | {k / 1e6:.1f} | "
                   f"{modo_transmision(res[clave]['cfg']['x0']):.0f} | {'>200' if bw is None else f'{bw:.0f}'} |")
    out["RESONANCIA"] = "\n".join(lin)
    out["UN_DETECTOR"] = f"error RMS un detector {err_1det[0]:.0f} nm, cuadratura {err_1det[1]:.1f} nm"
    # Validación: |G_i| y fase, estimado (hardware) contra verdad, corrida A
    lin = ["| f (Hz) | |G_i| verdad (dB) | estimado | ∠G_i verdad (°) | estimado |", "|---|---|---|---|---|"]
    for r, v in zip(res["A"]["hw"], res["A"]["verdad"]):
        lin.append(f"| {r['f']:g} | {an.db(v['Gi']):+.2f} | {an.db(r['Gi']):+.2f} | "
                   f"{an.fase_grad(v['Gi']):+.1f} | {an.fase_grad(r['Gi']):+.1f} |")
    out["VALIDACION"] = "\n".join(lin)
    return out


if __name__ == "__main__":
    SALIDA.mkdir(exist_ok=True)
    res = {}
    for i, (clave, cfg) in enumerate(CORRIDAS.items()):
        print(f"Corrida {clave} ({cfg['nombre']})", flush=True)
        r = correr_una(clave, cfg, seed=100 + i)
        r["hw"] = analizar(r, "hardware")[0]
        res[clave] = r
    sinc = {m: analizar(res["A"], m) for m in ("hardware", "marcas", "una_marca", "ninguno")}
    err_1det = figuras(res, sinc)
    for k, v in informe(res, sinc, err_1det).items():
        print(f"\n=== {k} ===\n{v}")
