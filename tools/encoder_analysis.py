"""Cuentas del barrido de caracterizacion del encoder MT6835 (solo numpy, sin graficos).

Las usan tools/encoder_sweep.py (linea de comandos, PNG con matplotlib) y la pestaña
«Encoder» de tools/interfaz.py. Las ventanas de 2 periodos son las de tools/plot_periods.py.

Muestras crudas (una fila por muestra), tal como las manda la placa:
    D,<t_us>,<sentido>,<campo comandado [rad electricos]>,<angulo crudo del chip>,<estado>
- firmware de barrido (variantes_main/main_barrido_encoder.cpp): comando G56
- firmware real: M57 X<mm> F<mm/s> P<n> R1 (una pasada por comando, t_us absoluto)
estado: bit0 sobrevelocidad, bit1 campo debil, bit2 subtension, bit3 error de CRC.
"""
import os

import numpy as np

COLS = ["t_s", "dir", "field_rad", "motor_deg", "x_mm", "raw", "enc_deg",
        "ideal_deg", "error_deg", "overspeed", "weak", "undervolt", "crc_err"]
CSV_FMT = ["%.6f", "%d", "%.5f", "%.4f", "%.4f", "%d", "%.4f", "%.4f", "%.4f", "%d", "%d", "%d", "%d"]
CPR = 2 ** 21
JUMP_DEG = 45.0          # salto: paso medido - esperado entre muestras seguidas
FLIP_DEG = 120.0         # muestra "corrida" (fuera de fase) en las ventanas de plot_periods.py
SETTLE_S = 1.0           # rampas de arranque/frenado que se descartan en las ventanas


def wrap180(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def nan_at_wraps(y):
    """Para no unir con una linea los puntos donde el angulo da la vuelta (+180 -> -180)."""
    y = np.array(y, dtype=float)
    if len(y) > 1:
        y[1:][np.abs(np.diff(y)) > 180] = np.nan
    return y


def parse_line(line):
    """'D,...' -> [t_us, dir, field, raw, status] o None."""
    if not line.startswith("D,"):
        return None
    p = line.split(",")
    if len(p) < 6:
        return None
    try:
        return [int(p[1]), int(p[2]), float(p[3]), int(p[4]), int(p[5])]
    except ValueError:
        return None


def parse_meta(line):
    """'META,k=v,...' -> dict de floats (lo que no es numero queda como texto)."""
    out = {}
    for tok in line.split(",")[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def build_table(raw_rows, pole_pairs, cpr, magnets, pitch_mm, field_ref=None):
    """Tabla con las columnas COLS. field_ref: campo [rad el.] que se toma como 0 de la
    posicion del motor (por defecto la primera muestra; la interfaz usa el del home)."""
    raw_rows = np.asarray(raw_rows, dtype=float)
    t_us, direction, field, raw, status = raw_rows.T
    f0 = field[0] if field_ref is None else field_ref
    motor_rad = (field - f0) / pole_pairs
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
        if not m.any():
            continue
        off = np.degrees(np.angle(np.mean(np.exp(1j * np.radians(enc[m] - s * phase_cmd[m])))))
        ideal[m] = s * phase_cmd[m] + off
    error = wrap180(enc - ideal)

    st = status.astype(int)
    circumference = magnets * pitch_mm
    table = np.column_stack([
        (t_us - t_us[0]) * 1e-6, direction, field, np.degrees(motor_rad), motor_rad / (2 * np.pi) * circumference,
        raw, enc, wrap180(ideal) % 360.0, error,
        st & 1, (st >> 1) & 1, (st >> 2) & 1, (st >> 3) & 1])
    return table, s


def columns(table):
    return {c: table[:, i] for i, c in enumerate(COLS)}


def save_csv(table, path):
    np.savetxt(path, table, delimiter=",", header=",".join(COLS), comments="", fmt=CSV_FMT)


def load_csv(path):
    return np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)


def jumps(T):
    """(t, paso medido - esperado, mascara de saltos) entre muestras seguidas del mismo sentido."""
    d_enc = wrap180(np.diff(T["enc_deg"]))
    d_ideal = wrap180(np.diff(T["ideal_deg"]))
    dev = wrap180(d_enc - d_ideal)
    mask = (np.abs(dev) > JUMP_DEG) & (np.diff(T["dir"]) == 0)
    return T["t_s"][1:], dev, mask


def summary(table, periods):
    """Los numeros que encoder_sweep.py imprime en la consola."""
    T = columns(table)
    fwd = T["dir"] > 0
    out = {
        "n": len(table),
        "weak_n": int(T["weak"].sum()), "weak_pct": 100 * float(T["weak"].mean()),
        "overspeed_n": int(T["overspeed"].sum()), "undervolt_n": int(T["undervolt"].sum()),
        "crc_n": int(T["crc_err"].sum()),
        "jumps": int(jumps(T)[2].sum()),
        "duration_s": float(T["t_s"][-1] - T["t_s"][0]) if len(table) else 0.0,
        "revs": float(np.ptp(T["motor_deg"]) / 360.0) if len(table) else 0.0,
    }
    for name, m in (("ida", fwd), ("vuelta", ~fwd)):
        e = T["error_deg"][m]
        out[name] = None if not len(e) else {
            "rms": float(np.sqrt(np.mean(e ** 2))), "max": float(np.max(np.abs(e))),
            "rms_shaft": float(np.sqrt(np.mean(e ** 2)) / periods)}
    out["periods_per_rev"] = float("nan")
    if fwd.sum() > 10 and np.ptp(T["motor_deg"][fwd]) > 1.0:
        unw = np.unwrap(np.radians(T["enc_deg"][fwd]))
        out["periods_per_rev"] = abs(float(np.polyfit(np.radians(T["motor_deg"][fwd]), unw, 1)[0]))
    return out


def steady_mask(T):
    """Descarta el primer y el ultimo SETTLE_S de cada sentido (rampas, como plot_periods.py)."""
    t = T["t_s"]
    steady = np.zeros(len(t), bool)
    for s in (+1, -1):
        m = T["dir"] == s
        if m.any():
            steady |= m & (t > t[m][0] + SETTLE_S) & (t < t[m][-1] - SETTLE_S)
    return steady


def window_geometry(magnets, pitch_mm, periods_per_window):
    circumference = magnets * pitch_mm
    win_mm = periods_per_window * 2 * pitch_mm
    return circumference, win_mm, int(round(circumference / win_mm))


def window_stats(table, magnets, pitch_mm, periods_per_window=2):
    """Una fila por ventana de la rueda y sentido (como la tabla de plot_periods.py):
    dict(w, dir, mag0, mag1, deg0, deg1, n, weak_pct, flips, rms)."""
    T = columns(table)
    circumference, win_mm, n_win = window_geometry(magnets, pitch_mm, periods_per_window)
    x = (T["motor_deg"] % 360.0) / 360.0 * circumference
    steady = steady_mask(T)
    rows = []
    for w in range(n_win):
        x0, x1 = w * win_mm, (w + 1) * win_mm
        for s in (+1, -1):
            sel = steady & (T["dir"] == s) & (x >= x0) & (x < x1)
            err = T["error_deg"][sel]
            ok = err[np.abs(err) < FLIP_DEG]
            rows.append(dict(
                w=w, dir=s, mag0=int(round(x0 / pitch_mm)), mag1=int(round(x1 / pitch_mm)) - 1,
                deg0=x0 / circumference * 360.0, deg1=x1 / circumference * 360.0, n=int(sel.sum()),
                weak_pct=100 * float(T["weak"][sel].mean()) if sel.any() else float("nan"),
                flips=int(np.sum(np.abs(err) >= FLIP_DEG)),
                rms=float(np.sqrt(np.mean(ok ** 2))) if len(ok) else float("nan")))
    return rows


def default_outdir(repo):
    import datetime
    return os.path.join(repo, "mediciones", datetime.date.today().isoformat())
