"""HP 8153A Lightwave Multimeter por GPIB, para la pestaña «HP 8153A» de la interfaz.

Lo que hay acá:
- HP8153A: los comandos SCPI del medidor de potencia (manual del 8153A, capítulos 6 y 8).
- Simulado: un 8153A de mentira que contesta los mismos comandos, para probar sin el equipo.
- HPLink: un hilo que habla con el equipo (abrir, configurar, leer en continuo) sin trabar la
  interfaz. Todo el GPIB pasa por ese hilo.
- HPHistory: las lecturas de los últimos minutos, para los gráficos.

GPIB desde Python: pyvisa (pip install pyvisa pyvisa-py) más el driver de la placa GPIB-USB-HS
(NI-488.2 / NI-VISA, o linux-gpib en Linux). Sin pyvisa solo anda el equipo simulado.

Cómo mide el 8153A (manual, págs. 8-4 a 8-12): cada lectura es el promedio de la potencia durante
el tiempo de promediación (SENS:POW:ATIM, de 20 ms a 3600 s). READ:POW? arranca una medición y
contesta al terminarla; con más de 1 s el equipo pasa a un promedio móvil que se renueva cada 1 s.
No tiene entrada de disparo externo.
"""

import bisect
import math
import queue
import random
import threading
import time

from PyQt5 import QtCore

try:
    import pyvisa
except ImportError:                      # sin pyvisa solo queda el simulado
    pyvisa = None

SIMULADO = "SIMULADO (sin equipo)"
DEFAULT_RESOURCE = "GPIB0::22::INSTR"   # 22: la dirección de fábrica del 8153A (manual, cap. 5)
TAVG_MS = (20, 50, 100, 200, 500, 1000)
UNIT_W, UNIT_DBM = 1, 0                  # como los devuelve SENS:POW:UNIT?
HISTORY_S = 600                          # lecturas que se guardan para los gráficos


def visa_disponible():
    return pyvisa is not None


# ======================================================================================
# Equipo
# ======================================================================================

class HP8153A:
    """Un canal (1 = A, 2 = B) de sensor del 8153A sobre un recurso VISA (o el Simulado)."""

    def __init__(self, res, canal=1):
        self.res = res
        self.canal = canal
        self.idn = ""
        self.modulos = []

    def q(self, cmd):
        return self.res.query(cmd).strip()

    def w(self, cmd):
        self.res.write(cmd)

    def identificar(self):
        """*IDN? y *OPT? (el módulo de cada canal, por ejemplo HP81531A,EMPTY)."""
        self.idn = self.q("*IDN?")
        try:
            self.modulos = [m.strip() for m in self.q("*OPT?").split(",")]
        except Exception:                # noqa: BLE001  algunos firmwares viejos podrían no tenerlo
            self.modulos = []
        return self.idn, self.modulos

    def leer_config(self):
        c = self.canal
        cfg = {"tavg_s": float(self.q(f"SENS{c}:POW:ATIM?")),
               "unidad": int(float(self.q(f"SENS{c}:POW:UNIT?"))),
               "autorango": bool(int(float(self.q(f"SENS{c}:POW:RANG:AUTO?")))),
               "rango_dbm": int(float(self.q(f"SENS{c}:POW:RANG?")))}
        try:
            cfg["lambda_nm"] = float(self.q(f"SENS{c}:POW:WAVE?")) * 1e9
        except ValueError:
            cfg["lambda_nm"] = float("nan")
        return cfg

    def aplicar_config(self, cfg):
        c = self.canal
        self.w(f"SENS{c}:POW:ATIM {int(round(cfg['tavg_s'] * 1000))}MS")
        self.w(f"SENS{c}:POW:UNIT {'W' if cfg['unidad'] == UNIT_W else 'DBM'}")
        if cfg.get("lambda_nm") and math.isfinite(cfg["lambda_nm"]):
            self.w(f"SENS{c}:POW:WAVE {cfg['lambda_nm']:.1f}NM")
        if cfg["autorango"]:
            self.w(f"SENS{c}:POW:RANG:AUTO ON")
        else:
            self.w(f"SENS{c}:POW:RANG:AUTO OFF")
            self.w(f"SENS{c}:POW:RANG {int(cfg['rango_dbm'])}DBM")
        # sin disparo continuo: READ:POW? dispara su propia medición y no hay lecturas repetidas
        self.w(f"INIT{c}:CONT OFF")
        self.q("*OPC?")                  # esperar a que el equipo termine de tomar todo
        return self.leer_config()

    def leer(self):
        """Una medición completa: tarda lo que el tiempo de promediación (más el GPIB)."""
        return float(self.q(f"READ{self.canal}:POW?"))


class Simulado:
    """Contesta como un 8153A con un HP 81531A en el canal A: potencia de unos 12 µW que deriva
    despacio, con ruido, y tarda el tiempo de promediación en cada lectura."""

    def __init__(self):
        self.t0 = time.monotonic()
        self.state = {1: dict(atim=0.2, unit=UNIT_W, auto=1, rang=-10, wave=1550e-9),
                      2: dict(atim=0.2, unit=UNIT_W, auto=1, rang=-10, wave=1550e-9)}
        self._out = ""
        self.timeout = 5000

    def _potencia_w(self, atim):
        t = time.monotonic() - self.t0
        p = 12e-6 * (1 + 0.05 * math.sin(2 * math.pi * t / 20) + 0.01 * math.sin(2 * math.pi * t / 3.1))
        return p * (1 + random.gauss(0, 2e-3 / math.sqrt(max(atim, 0.02) / 0.02)))

    def write(self, cmd):
        cmd = cmd.strip().upper()
        ch = 2 if cmd[4:5] == "2" else 1
        st = self.state[ch]
        arg = cmd.split(" ", 1)[1] if " " in cmd else ""
        if cmd == "*IDN?":
            self._out = "HEWLETT-PACKARD,HP8153A,SIMULADO,0.0"
        elif cmd == "*OPT?":
            self._out = "HP81531A,EMPTY"
        elif cmd == "*OPC?":
            self._out = "1"
        elif ":ATIM?" in cmd:
            self._out = f"{st['atim']:+.5E}"
        elif ":ATIM" in cmd:
            v = float(arg.replace("MS", "")) / (1000 if "MS" in arg else 1)
            st["atim"] = min(3600.0, max(0.02, v))
        elif ":UNIT?" in cmd:
            self._out = f"{st['unit']:+d}"
        elif ":UNIT" in cmd:
            st["unit"] = UNIT_W if arg.startswith("W") or arg == "1" else UNIT_DBM
        elif ":RANG:AUTO?" in cmd:
            self._out = f"{st['auto']:+d}"
        elif ":RANG:AUTO" in cmd:
            st["auto"] = 0 if arg in ("OFF", "0") else 1
        elif ":RANG?" in cmd:
            self._out = f"{st['rang']:+d}"
        elif ":RANG" in cmd:
            st["rang"] = int(round(float(arg.replace("DBM", "")) / 10) * 10)
        elif ":WAVE?" in cmd:
            self._out = f"{st['wave']:+.5E}"
        elif ":WAVE" in cmd:
            st["wave"] = float(arg.replace("NM", "")) * 1e-9
        elif cmd.startswith("READ") and cmd.endswith("POW?"):
            time.sleep(st["atim"] + 0.004)
            p = self._potencia_w(st["atim"])
            self._out = f"{p:+.5E}" if st["unit"] == UNIT_W else f"{10 * math.log10(p / 1e-3):+.5E}"

    def query(self, cmd):
        self.write(cmd)
        out, self._out = self._out, ""
        return out

    def close(self):
        pass


# ======================================================================================
# Hilo de comunicación
# ======================================================================================

class HPLink(QtCore.QObject):
    """Habla con el equipo desde un hilo propio. Las órdenes (listar, conectar, configurar) se
    encolan; entre orden y orden, si la lectura continua está activa, toma una lectura detrás de
    otra. Las lecturas quedan en una lista que la interfaz retira con take_rows()."""

    resources = QtCore.pyqtSignal(list)         # recursos VISA encontrados
    connected = QtCore.pyqtSignal(dict)         # idn, modulos, config
    configured = QtCore.pyqtSignal(dict)        # config leída del equipo después de aplicar
    disconnected = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.rm = None
        self.res = None
        self.hp = None
        self.resource = None
        self.reading = False
        self._rows = []
        self._rows_lock = threading.Lock()
        self._jobs = queue.Queue()
        self._alive = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    @property
    def is_open(self):
        return self.hp is not None

    # --- órdenes (se llaman desde la interfaz) ------------------------------------------
    def list_resources(self):
        self._jobs.put(self._do_list)

    def open(self, resource, canal):
        self._jobs.put(lambda: self._do_open(resource, canal))

    def close(self):
        self.reading = False
        self._jobs.put(self._do_close)

    def configure(self, cfg):
        self._jobs.put(lambda: self._do_configure(cfg))

    def set_reading(self, on):
        self.reading = bool(on) and self.hp is not None

    def take_rows(self):
        """Lecturas desde la última llamada: (t_pedido, t_respuesta, hora_unix, valor). Los dos
        primeros son time.monotonic() de la PC: el pedido salió en t_pedido y la respuesta llegó
        en t_respuesta; la potencia es el promedio de un tramo de tavg_s entre esos dos."""
        with self._rows_lock:
            rows, self._rows = self._rows, []
        return rows

    def shutdown(self):
        self.reading = False
        self._jobs.put(self._do_close)
        self._jobs.put(None)
        self._thread.join(timeout=3.0)

    # --- en el hilo -----------------------------------------------------------------------
    def _loop(self):
        while True:
            try:
                job = self._jobs.get(timeout=0.0 if self.reading else 0.1)
            except queue.Empty:
                job = False
            if job is None:
                return
            if job:
                try:
                    job()
                except Exception as e:   # noqa: BLE001  el error se muestra en la pestaña
                    self.error.emit(self._msg(e))
                continue
            if self.reading and self.hp is not None:
                try:
                    t0 = time.monotonic()
                    v = self.hp.leer()
                    t1 = time.monotonic()
                    with self._rows_lock:
                        self._rows.append((t0, t1, time.time(), v))
                except Exception as e:   # noqa: BLE001
                    self.reading = False
                    self.error.emit("Falló una lectura, se detuvo la lectura continua: " + self._msg(e))

    @staticmethod
    def _msg(e):
        text = str(e) or type(e).__name__
        return text.splitlines()[0][:300]

    def _manager(self):
        if self.rm is None:
            if pyvisa is None:
                raise RuntimeError("falta pyvisa (pip install pyvisa pyvisa-py): solo anda el simulado")
            try:
                self.rm = pyvisa.ResourceManager()          # NI-VISA u otra VISA instalada
            except (OSError, ValueError):
                self.rm = pyvisa.ResourceManager("@py")     # pyvisa-py (GPIB con linux-gpib)
        return self.rm

    def _do_list(self):
        found = []
        if pyvisa is not None:
            try:
                found = list(self._manager().list_resources("GPIB?*INSTR"))
            except Exception as e:       # noqa: BLE001
                self.error.emit("No se pudieron listar los recursos VISA: " + self._msg(e))
        self.resources.emit(found)

    def _do_open(self, resource, canal):
        self._do_close(quiet=True)
        if resource == SIMULADO:
            res = Simulado()
        else:
            res = self._manager().open_resource(resource)
            res.read_termination = "\n"
            res.write_termination = "\n"
            res.timeout = 5000
            try:
                res.clear()                # Device Clear: vacía lo que haya quedado en el equipo
            except Exception:            # noqa: BLE001
                pass
        hp = HP8153A(res, canal)
        try:
            idn, modulos = hp.identificar()
            cfg = hp.leer_config()
        except Exception:
            res.close()
            raise
        self._set_timeout(res, cfg["tavg_s"])
        self.res, self.hp, self.resource = res, hp, resource
        self.connected.emit({"idn": idn, "modulos": modulos, "config": cfg, "canal": canal,
                             "recurso": resource})

    def _do_configure(self, cfg):
        if self.hp is None:
            return
        was = self.reading
        self.reading = False
        try:
            cfg = self.hp.aplicar_config(cfg)
            self._set_timeout(self.res, cfg["tavg_s"])
            with self._rows_lock:
                self._rows = []          # lecturas con la configuración vieja (otra unidad, otro rango)
            self.configured.emit(cfg)
        finally:
            self.reading = was

    @staticmethod
    def _set_timeout(res, tavg_s):
        res.timeout = int(min(tavg_s, 1.0) * 1000 + 3000)   # con más de 1 s el equipo contesta cada 1 s

    def _do_close(self, quiet=False):
        self.reading = False
        if self.res is not None:
            try:
                self.res.close()
            except Exception:            # noqa: BLE001
                pass
        was_open = self.hp is not None
        self.res = self.hp = self.resource = None
        if was_open and not quiet:
            self.disconnected.emit()


# ======================================================================================
# Historia para los gráficos
# ======================================================================================

class HPHistory:
    """Lecturas de los últimos HISTORY_S segundos: t (respuesta, time.monotonic()) y valor."""

    def __init__(self):
        self.t = []
        self.v = []

    def clear(self):
        self.t, self.v = [], []

    def add(self, rows):
        for _, t1, _, v in rows:
            self.t.append(t1)
            self.v.append(v)
        if self.t and self.t[-1] - self.t[0] > HISTORY_S * 1.2:
            cut = bisect.bisect_left(self.t, self.t[-1] - HISTORY_S)
            del self.t[:cut], self.v[:cut]

    def last(self, seconds, now):
        i = bisect.bisect_left(self.t, now - seconds)
        return self.t[i:], self.v[i:]
