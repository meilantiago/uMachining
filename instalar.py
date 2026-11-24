#!/usr/bin/env python3
"""Instalador de μMachining: crea un entorno de Python, instala la interfaz y sus dependencias,
y avisa cuando terminó.

    python3 instalar.py                 (Windows: doble clic en instalar.bat, o  py instalar.py)
    python3 instalar.py --entorno DIR   otra carpeta para el entorno (por defecto ~/umachining-venv)
    python3 instalar.py --demo          solo muestra el mensaje final

Solo usa la biblioteca estándar de Python (3.8 o más nuevo).
"""
import argparse
import glob
import math
import os
import platform
import shutil
import subprocess
import sys
import time

URL = "https://github.com/meilantiago/uMachining/archive/refs/heads/Firmware-para-usuarios.zip"
AQUI = os.path.dirname(os.path.abspath(__file__))
WINDOWS = os.name == "nt"
LINUX = sys.platform.startswith("linux")
ARM = platform.machine().lower() in ("aarch64", "arm64", "armv7l", "armv6l") and LINUX
PASOS = 4

# --- logo de TECSCI (Technology for Science) ----------------------------------------------
# «TEC» va en el color del texto de la terminal (negro en fondo claro, blanco en fondo oscuro) y
# «SCI» y la red de fondo en el turquesa del logo.

LETRAS = {
    "T": ["█████████",
          "█████████",
          "   ███   ",
          "   ███   ",
          "   ███   ",
          "   ███   "],
    "E": ["████████",
          "███     ",
          "███▄▄▄▄ ",
          "███▀▀▀▀ ",
          "███     ",
          "████████"],
    "C": [" ▄███████",
          "███▀     ",
          "███      ",
          "███      ",
          "███▄     ",
          " ▀███████"],
    "S": [" ▄███████",
          "███▀     ",
          "▀████▄▄  ",
          "  ▀▀▀████",
          "     ▄███",
          "███████▀ "],
    "I": ["████",
          "████",
          "████",
          "████",
          "████",
          "████"],
}
SUBTITULO = "Technology for Science"
MARGEN = 6                     # columnas de red a cada lado de las letras
# ondas de la red: centro [filas], amplitud [filas], longitud de onda [columnas], fase, columnas con nodo
ONDAS = [(1.5, 1.0, 70, 0.4, (5, 40, 68)),
         (3.8, 1.3, 60, 2.6, (12, 50)),
         (2.6, 1.6, 44, 4.4, (2, 30, 63))]



def armar_logo():
    """Capas del logo: letras (con su parte TEC/SCI), red de fondo y recorrido de cada onda."""
    alto = 6
    letras = [[None] * MARGEN for _ in range(alto)]
    for i, l in enumerate("TECSCI"):
        parte = "tec" if i < 3 else "sci"
        for r in range(alto):
            letras[r] += [(ch, parte) if ch != " " else None for ch in LETRAS[l][r]]
            if i < 5:
                letras[r].append(None)
    for r in range(alto):
        letras[r] += [None] * MARGEN
    ancho = len(letras[0])

    red = [[None] * ancho for _ in range(alto)]
    caminos = []
    for centro, amp, lam, fase, nodos in ONDAS:
        filas = [min(alto - 1, max(0, round(centro + amp * math.sin(2 * math.pi * x / lam + fase))))
                 for x in range(ancho)]
        camino = []
        for x, r in enumerate(filas):
            sig = filas[x + 1] if x + 1 < ancho else r
            if sig == r:
                celdas = [(r, "•" if x in nodos else "─")]
            elif sig > r:                    # baja: ╮ arriba, ╰ abajo, con │ si salta más de una fila
                celdas = [(r, "╮")] + [(k, "│") for k in range(r + 1, sig)] + [(sig, "╰")]
            else:                            # sube: ╯ abajo, ╭ arriba
                celdas = [(r, "╯")] + [(k, "│") for k in range(r - 1, sig, -1)] + [(sig, "╭")]
            for rr, ch in celdas:
                red[rr][x] = ch
                camino.append((rr, x))
        caminos.append(camino)
    return letras, red, caminos, ancho, alto


def color_turquesa():
    vivo = (os.environ.get("COLORTERM", "").lower() in ("truecolor", "24bit")
            or os.environ.get("WT_SESSION") or os.name == "nt")
    return "38;2;102;204;207" if vivo else "38;5;80"


def dibujar(letras, red, caminos, ancho, alto, ver_red, ver_letras, ver_sub, brillo=None, pulso=None,
            color=True, turquesa="38;5;80"):
    pulsos = set()
    if pulso is not None:
        for k, camino in enumerate(caminos):
            i = int(pulso * len(camino)) - 3 * k
            for j in (i - 1, i):
                if 0 <= j < len(camino):
                    pulsos.add(camino[j])
    filas = []
    for r in range(alto):
        s, actual = ["  "], None
        for c in range(ancho):
            ch, cod = " ", None
            if c < ver_letras and letras[r][c]:
                ch, parte = letras[r][c]
                en_brillo = brillo is not None and abs(c - brillo) <= 1
                if parte == "tec":
                    cod = ("1;" + turquesa) if en_brillo else "1;39"
                else:
                    cod = "1;97" if en_brillo else turquesa
            elif c < ver_red and red[r][c]:
                ch = red[r][c]
                cod = "1;97" if (r, c) in pulsos else (("1;" + turquesa) if ch == "•" else "22;" + turquesa)
            if color and cod != actual and ch != " ":
                s.append(f"\x1b[0;{cod}m")
                actual = cod
            s.append(ch)
        filas.append("".join(s) + ("\x1b[0m\x1b[K" if color else ""))
    sub = " ".join(SUBTITULO)
    pad = (ancho - len(sub)) // 2
    filas.append("  " + " " * pad + sub[:ver_sub] + ("\x1b[0m\x1b[K" if color else ""))
    return filas


# --- salida en la terminal ----------------------------------------------------------------

def _hay_color():
    if os.environ.get("NO_COLOR") or not sys.stdout.isatty():
        return False
    if WINDOWS:
        os.system("")            # activa las secuencias ANSI en la consola de Windows 10/11
    return True


COLOR = _hay_color()


def _puede(texto):
    try:
        texto.encode(sys.stdout.encoding or "utf-8")
        return True
    except (UnicodeEncodeError, LookupError):
        return False


def estilo(texto, codigo):
    return f"\x1b[{codigo}m{texto}\x1b[0m" if COLOR else texto


def paso(n, texto):
    print("\n" + estilo(f"[{n}/{PASOS}] {texto}", "1;36"), flush=True)


def ok(texto):
    marca = "✔" if _puede("✔") else "OK"
    print(estilo(f"  {marca} {texto}", "32"), flush=True)


def aviso(texto):
    print(estilo(f"  ! {texto}", "33"), flush=True)


def error(texto):
    marca = "✘" if _puede("✘") else "ERROR:"
    print("\n" + estilo(f"{marca} {texto}", "1;31"), flush=True)


def animacion():
    letras, red, caminos, ancho, alto = armar_logo()
    n_sub = len(" ".join(SUBTITULO))
    muestra = "".join(LETRAS["S"]) + "╭╮╯╰─│•"
    if not _puede(muestra):
        print("\n  TECSCI - Technology for Science")
        return
    if not COLOR or shutil.get_terminal_size((80, 24)).columns < ancho + 4:
        print()
        print("\n".join(dibujar(letras, red, caminos, ancho, alto, ancho, ancho, n_sub, color=False)))
        return
    tq = color_turquesa()
    total = alto + 1

    def cuadro(**kw):
        sys.stdout.write(f"\x1b[{total}A" + "\n".join(dibujar(letras, red, caminos, ancho, alto,
                                                              turquesa=tq, **kw)) + "\n")
        sys.stdout.flush()

    sys.stdout.write("\n\x1b[?25l" + "\n" * total)        # esconde el cursor y reserva el lugar
    try:
        for v in range(0, ancho + 1, 2):                       # 1. se dibuja la red
            cuadro(ver_red=v, ver_letras=0, ver_sub=0)
            time.sleep(0.012)
        for v in range(0, ancho + 1):                          # 2. aparecen las letras
            cuadro(ver_red=ancho, ver_letras=v, ver_sub=0)
            time.sleep(0.011)
        for v in range(0, n_sub + 1):                          # 3. se escribe el subtítulo
            cuadro(ver_red=ancho, ver_letras=ancho, ver_sub=v)
            time.sleep(0.02)
        pasos = 60                                             # 4. brillo y señales por la red
        for i in range(pasos + 1):
            cuadro(ver_red=ancho, ver_letras=ancho, ver_sub=n_sub,
                   brillo=-3 + (ancho + 6) * i / pasos, pulso=i / pasos)
            time.sleep(0.014)
        cuadro(ver_red=ancho, ver_letras=ancho, ver_sub=n_sub)
    finally:
        sys.stdout.write("\x1b[0m\x1b[?25h")
        sys.stdout.flush()


def final(entorno):
    animacion()
    print()
    marca = "✔" if _puede("✔") else ">>"
    print(estilo(f"  {marca}  Instalado correctamente", "1;" + color_turquesa()))
    print()
    print("  Para abrir la interfaz:")
    print("     " + estilo(_comillas(_ejecutable(entorno, "umachining")), "1"))
    print("  Para ver el firmware incluido y las placas conectadas:")
    print("     " + _comillas(_ejecutable(entorno, "umachining-flashear")) + " --info")
    print("  O activar el entorno y usar los comandos directamente:")
    print("     " + (_comillas(os.path.join(entorno, "Scripts", "activate")) if WINDOWS
                     else "source " + _comillas(os.path.join(entorno, "bin", "activate"))))
    print("  Guía completa: README.md")
    print()


# --- instalación --------------------------------------------------------------------------

def _comillas(p):
    return f'"{p}"' if " " in p else p


def _bin(entorno):
    return os.path.join(entorno, "Scripts" if WINDOWS else "bin")


def _python(entorno):
    return os.path.join(_bin(entorno), "python.exe" if WINDOWS else "python")


def _ejecutable(entorno, nombre):
    return os.path.join(_bin(entorno), nombre + (".exe" if WINDOWS else ""))


def correr(cmd):
    return subprocess.run(cmd).returncode == 0


def origen(pedido):
    if pedido:
        return pedido
    if os.path.exists(os.path.join(AQUI, "setup.cfg")) and os.path.isdir(os.path.join(AQUI, "umachining")):
        return AQUI              # instalador corrido desde una copia del repositorio
    return URL


def crear_entorno(entorno):
    if os.path.exists(_python(entorno)):
        ok(f"ya existía, se reutiliza: {entorno}")
        return True
    cmd = [sys.executable, "-m", "venv", entorno]
    if ARM:
        cmd.insert(3, "--system-site-packages")       # PyQt5 viene de apt en ARM
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0 or not os.path.exists(_python(entorno)):
        shutil.rmtree(entorno, ignore_errors=True)
        error("No se pudo crear el entorno de Python.")
        print((r.stderr or r.stdout).strip())
        if LINUX:
            v = f"{sys.version_info.major}.{sys.version_info.minor}"
            print(f"\nEn Ubuntu/Debian falta un paquete: sudo apt install python3-venv  (o python{v}-venv)")
        return False
    ok(f"creado en {entorno}")
    return True


def revisar_arm():
    if not ARM:
        return True
    r = subprocess.run([sys.executable, "-c", "import PyQt5, pyqtgraph"], capture_output=True)
    if r.returncode == 0:
        return True
    error("En Linux ARM (Raspberry Pi) PyQt5 no se instala con pip.")
    print("Instalarlo con apt y volver a correr el instalador:\n"
          "  sudo apt install python3-pyqt5 python3-pyqtgraph")
    return False


def permisos_puerto():
    """Linux: el usuario tiene que poder abrir /dev/ttyACM*."""
    if not LINUX:
        ok("no hace falta configurar permisos en este sistema")
        return
    try:
        import grp
        import pwd
        usuario = pwd.getpwuid(os.getuid()).pw_name
        g = grp.getgrnam("dialout")
        en_dialout = g.gr_gid in os.getgroups() or usuario in g.gr_mem
    except (ImportError, KeyError):
        ok("no hay grupo dialout: no hace falta configurar nada")
        return
    accesibles = [p for p in glob.glob("/dev/ttyACM*") if os.access(p, os.R_OK | os.W_OK)]
    if en_dialout or accesibles or os.path.exists("/etc/udev/rules.d/99-platformio-udev.rules"):
        ok("el usuario puede abrir el puerto serie de la placa")
        return
    aviso(f"el usuario '{usuario}' no está en el grupo dialout: no va a poder abrir el puerto de la placa.")
    print(f"    Se arregla con:  sudo usermod -aG dialout {usuario}   (y después cerrar sesión y volver a entrar)")
    try:
        r = input("    ¿Ejecutarlo ahora? Pide la contraseña de la PC. [s/N] ").strip().lower()
    except EOFError:
        r = ""
    if r in ("s", "si", "sí", "y", "yes"):
        if correr(["sudo", "usermod", "-aG", "dialout", usuario]):
            ok("listo: cerrar la sesión y volver a entrar para que tome efecto")
        else:
            aviso("no se pudo; correr el comando a mano")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--entorno", default=os.path.join(os.path.expanduser("~"), "umachining-venv"),
                    help="carpeta del entorno de Python (por defecto ~/umachining-venv)")
    ap.add_argument("--desde", help="instalar desde esta carpeta o URL (por defecto, la copia local o GitHub)")
    ap.add_argument("--demo", action="store_true", help="solo mostrar el mensaje final")
    args = ap.parse_args()
    entorno = os.path.abspath(os.path.expanduser(args.entorno))

    if args.demo:
        final(entorno)
        return 0

    print(estilo("Instalador de μMachining" if _puede("μ") else "Instalador de uMachining", "1"))

    paso(1, "Revisando Python")
    if sys.version_info < (3, 8):
        error(f"Hace falta Python 3.8 o más nuevo y este es {platform.python_version()}.")
        return 1
    ok(f"Python {platform.python_version()} ({sys.executable})")
    if not revisar_arm():
        return 1

    paso(2, "Creando el entorno de Python")
    if not crear_entorno(entorno):
        return 1

    src = origen(args.desde)
    paso(3, "Instalando la interfaz y sus dependencias (puede tardar unos minutos)")
    print(f"  desde {src}\n", flush=True)
    py = _python(entorno)
    correr([py, "-m", "pip", "install", "--upgrade", "pip"])
    subprocess.run([py, "-m", "pip", "uninstall", "-y", "-q", "umachining"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)   # para reinstalar si ya estaba
    if not correr([py, "-m", "pip", "install", src]):
        error("Falló la instalación con pip (ver los mensajes de arriba).")
        print("Revisar la conexión a internet y volver a correr el instalador. Si sigue fallando, ver\n"
              "«Si algo no funciona» en el README.")
        return 1
    r = subprocess.run([py, "-c", "import umachining, PyQt5, pyqtgraph, serial, numpy, matplotlib; "
                                  "print(umachining.__version__)"], capture_output=True, text=True)
    if r.returncode != 0:
        error("El paquete quedó instalado pero no se puede importar:")
        print(r.stderr.strip())
        return 1
    ok(f"umachining {r.stdout.strip()} instalado")

    paso(4, "Permisos del puerto USB")
    permisos_puerto()

    final(entorno)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.stdout.write("\x1b[0m\x1b[?25h" if COLOR else "")
        print("\nInstalación cancelada.")
        sys.exit(1)
