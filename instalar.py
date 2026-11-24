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

ARTE = [
    "████████╗███████╗ ██████╗███████╗ ██████╗██╗",
    "╚══██╔══╝██╔════╝██╔════╝██╔════╝██╔════╝██║",
    "   ██║   █████╗  ██║     ███████╗██║     ██║",
    "   ██║   ██╔══╝  ██║     ╚════██║██║     ██║",
    "   ██║   ███████╗╚██████╗███████║╚██████╗██║",
    "   ╚═╝   ╚══════╝ ╚═════╝╚══════╝ ╚═════╝╚═╝",
]
ARTE_ASCII = [            # si la terminal no puede mostrar los caracteres de bloque
    "########  #######   ######   ######   ######  ##",
    "   ##     ##       ##       ##       ##       ##",
    "   ##     #####    ##        #####   ##       ##",
    "   ##     ##       ##            ##  ##       ##",
    "   ##     #######   ######  ######    ######  ##",
]
DEGRADE = [51, 45, 39, 33, 63, 99, 135, 171, 207, 213]   # cian -> azul -> violeta -> rosa (256 colores)


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
    arte = ARTE if _puede("".join(ARTE)) else ARTE_ASCII
    ancho, alto = max(len(l) for l in arte), len(arte)
    arte = [l.ljust(ancho) for l in arte]
    if not COLOR:
        print("\n" + "\n".join("  " + l for l in arte))
        return

    def color(c):
        return DEGRADE[min(len(DEGRADE) - 1, c * len(DEGRADE) // ancho)]

    def cuadro(visible, brillo=None):
        out = [f"\x1b[{alto}A"]
        for linea in arte:
            s = ["  "]
            for c, ch in enumerate(linea):
                if c >= visible or ch == " ":
                    s.append(" ")
                elif brillo is not None and abs(c - brillo) <= 2:
                    s.append(f"\x1b[1;97m{ch}")
                else:
                    s.append(f"\x1b[22;38;5;{color(c)}m{ch}")
            out.append("".join(s) + "\x1b[0m\x1b[K\n")
        sys.stdout.write("".join(out))
        sys.stdout.flush()

    sys.stdout.write("\x1b[?25l" + "\n" * alto)       # esconde el cursor y reserva el lugar
    try:
        for visible in range(0, ancho + 1, 2):         # aparece de izquierda a derecha
            cuadro(visible)
            time.sleep(0.025)
        for _ in range(2):                             # dos pasadas de brillo
            for b in range(-3, ancho + 4, 2):
                cuadro(ancho, b)
                time.sleep(0.012)
        cuadro(ancho)
    finally:
        sys.stdout.write("\x1b[0m\x1b[?25h")
        sys.stdout.flush()


def final(entorno):
    animacion()
    print()
    marca = "✔" if _puede("✔") else ">>"
    print(estilo(f"  {marca}  Instalado correctamente", "1;32"))
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
