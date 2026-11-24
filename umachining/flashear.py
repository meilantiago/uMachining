"""Graba el firmware del eje en la Raspberry Pi Pico (RP2040), sin PlatformIO.

    umachining-flashear              graba el firmware que viene con el paquete
    umachining-flashear --info       muestra qué firmware trae el paquete y qué placas ve, sin grabar
    umachining-flashear --uf2 X.uf2  graba otro .uf2 (por ejemplo uno compilado con PlatformIO)

Cómo funciona: la Pico, en modo BOOTSEL, aparece como una unidad USB llamada RPI-RP2. Copiar
un .uf2 a esa unidad la graba y la reinicia. Si la placa ya tiene un firmware de este proyecto y
está conectada por USB, el programa la pasa sola a modo BOOTSEL (reinicio a 1200 baudios). Si no
(placa nueva, sin firmware o colgada), hay que hacerlo a mano: desenchufar, mantener apretado el
botón BOOTSEL, enchufar y soltar.

Después de grabar, la calibración que estaba en RAM se pierde: hay que correr M56 (en la
interfaz, «Calibrar e iniciar») antes de mover el eje.
"""
import argparse
import glob
import hashlib
import os
import string
import struct
import sys
import time

import serial

from . import puertos

AQUI = os.path.dirname(os.path.abspath(__file__))
UF2_INCLUIDO = os.path.join(AQUI, "firmware", "firmware.uf2")
SUMAS = os.path.join(AQUI, "firmware", "SHA256SUMS")
INFO = os.path.join(AQUI, "firmware", "INFO.txt")

UF2_MAGIC = (0x0A324655, 0x9E5D5157)
FAMILIA_RP2040 = 0xE48BFF56


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for bloque in iter(lambda: f.read(1 << 16), b""):
            h.update(bloque)
    return h.hexdigest()


def sha_esperado(nombre="firmware.uf2"):
    try:
        with open(SUMAS) as f:
            for linea in f:
                partes = linea.split()
                if len(partes) == 2 and partes[1].lstrip("*") == nombre:
                    return partes[0]
    except OSError:
        pass
    return None


def revisar_uf2(path):
    """Comprueba que sea un UF2 para RP2040. Devuelve la cantidad de bloques."""
    with open(path, "rb") as f:
        datos = f.read()
    if not datos or len(datos) % 512:
        raise ValueError("no es un archivo UF2 (el tamaño no es múltiplo de 512 bytes)")
    familias = set()
    for i in range(0, len(datos), 512):
        m0, m1, flags, _, _, _, _, familia = struct.unpack_from("<8I", datos, i)
        if (m0, m1) != UF2_MAGIC:
            raise ValueError(f"no es un archivo UF2 (bloque {i // 512} sin la firma UF2)")
        if flags & 0x2000:
            familias.add(familia)
    if familias and familias != {FAMILIA_RP2040}:
        raise ValueError("el UF2 no es para RP2040 (familias: "
                         + ", ".join(f"0x{x:08x}" for x in sorted(familias)) + ")")
    return len(datos) // 512


# --- la unidad RPI-RP2 ------------------------------------------------------------------

def _carpetas_posibles():
    if sys.platform.startswith("win"):
        return [f"{letra}:\\" for letra in string.ascii_uppercase]
    if sys.platform == "darwin":
        return glob.glob("/Volumes/*")
    user = os.environ.get("USER", "*")
    pats = [f"/media/{user}/*", f"/run/media/{user}/*", "/media/*/*", "/run/media/*/*", "/media/*", "/mnt/*"]
    out = []
    for p in pats:
        out += glob.glob(p)
    return list(dict.fromkeys(out))


def leer_info_uf2(carpeta):
    try:
        with open(os.path.join(carpeta, "INFO_UF2.TXT"), errors="replace") as f:
            return f.read()
    except OSError:
        return None


def buscar_unidades():
    """Carpetas montadas que son una placa en modo BOOTSEL (tienen INFO_UF2.TXT)."""
    out = []
    for c in _carpetas_posibles():
        info = leer_info_uf2(c)
        if info is not None:
            out.append((c, info))
    return out


def esperar_unidad(segundos, mensaje_cada=None):
    t0 = time.time()
    aviso = t0
    while time.time() - t0 < segundos:
        u = buscar_unidades()
        if u:
            return u
        if mensaje_cada and time.time() - aviso > mensaje_cada:
            print(f"  … esperando la unidad RPI-RP2 ({int(segundos - (time.time() - t0))} s más)")
            aviso = time.time()
        time.sleep(0.5)
    return []


def reinicio_a_bootsel(port):
    """Abrir el puerto a 1200 baudios y cerrarlo: el core de Arduino de la Pico lo toma como pedido
    de reinicio al cargador (BOOTSEL), igual que hace PlatformIO antes de grabar."""
    try:
        s = serial.Serial(port, 1200)
        try:
            s.dtr = False
        except (OSError, serial.SerialException):
            pass
        time.sleep(0.2)
        s.close()
    except (OSError, serial.SerialException) as e:
        print(f"  no se pudo abrir {port} ({e})")
        return False
    return True


def copiar(uf2, carpeta):
    destino = os.path.join(carpeta, "firmware.uf2")
    with open(uf2, "rb") as f:
        datos = f.read()
    try:
        with open(destino, "wb") as f:
            f.write(datos)
            f.flush()
            os.fsync(f.fileno())
    except OSError as e:
        # La placa se reinicia apenas recibe el último bloque y la unidad desaparece: en algunos
        # sistemas eso llega como error al cerrar el archivo aunque la grabación haya terminado.
        print(f"  aviso al cerrar el archivo ({e}); se verifica viendo si la placa vuelve a aparecer")


def esperar_puerto(segundos):
    t0 = time.time()
    while time.time() - t0 < segundos:
        p = puertos.picos()
        if p:
            return p[0]
        time.sleep(0.5)
    return None


def preguntar(texto):
    try:
        return input(texto + " [s/N] ").strip().lower() in ("s", "si", "sí", "y", "yes")
    except EOFError:
        return False


# --- programa -----------------------------------------------------------------------------

def info():
    print("Firmware incluido en el paquete:")
    if os.path.exists(INFO):
        with open(INFO, encoding="utf-8") as f:
            print("  " + f.read().strip().replace("\n", "\n  "))
    if os.path.exists(UF2_INCLUIDO):
        print(f"  Archivo: {UF2_INCLUIDO}")
        real, esp = sha256(UF2_INCLUIDO), sha_esperado()
        print(f"  SHA-256: {real}" + ("  (verificado)" if real == esp else "  (NO coincide con SHA256SUMS)"))
    else:
        print("  ¡no está el firmware.uf2!")
    print("\nPlacas por USB con firmware (puerto serie):")
    for p in puertos.picos() or ["  ninguna"]:
        print("  " + p)
    print("\nPlacas en modo BOOTSEL (unidad RPI-RP2):")
    unidades = buscar_unidades()
    for c, i in unidades:
        print(f"  {c}  ({_board_id(i)})")
    if not unidades:
        print("  ninguna")


def _board_id(info_txt):
    for linea in info_txt.splitlines():
        if linea.lower().startswith("board-id"):
            return linea.split(":", 1)[1].strip()
    return "placa UF2"


def main(argv=None):
    ap = argparse.ArgumentParser(prog="umachining-flashear", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uf2", help="otro archivo .uf2 (por defecto, el que trae el paquete)")
    ap.add_argument("--unidad", help="carpeta donde está montada la unidad RPI-RP2, si no la encuentra sola")
    ap.add_argument("--puerto", help="puerto serie de la placa, para pasarla sola a BOOTSEL")
    ap.add_argument("--sin-reinicio", action="store_true",
                    help="no intentar pasar la placa a BOOTSEL por el puerto serie")
    ap.add_argument("--espera", type=float, default=180, help="segundos a esperar la unidad RPI-RP2 (180)")
    ap.add_argument("-s", "--si", action="store_true", help="no pedir confirmación")
    ap.add_argument("--info", action="store_true", help="mostrar el firmware incluido y las placas conectadas")
    args = ap.parse_args(argv)

    if args.info:
        info()
        return 0

    uf2 = os.path.abspath(args.uf2) if args.uf2 else UF2_INCLUIDO
    if not os.path.exists(uf2):
        print(f"No existe {uf2}")
        return 1
    try:
        bloques = revisar_uf2(uf2)
    except ValueError as e:
        print(f"{uf2}: {e}")
        return 1
    if not args.uf2:
        real, esp = sha256(uf2), sha_esperado()
        if esp and real != esp:
            print(f"El firmware.uf2 del paquete no coincide con SHA256SUMS:\n  {real}\n  {esp}\n"
                  "Reinstalar el paquete.")
            return 1
        print(f"Firmware: {uf2}\n  SHA-256 {real[:16]}… verificado, {bloques} bloques UF2")
    else:
        print(f"Firmware: {uf2}\n  SHA-256 {sha256(uf2)[:16]}…, {bloques} bloques UF2")

    # 1. la unidad
    if args.unidad:
        if leer_info_uf2(args.unidad) is None:
            print(f"{args.unidad} no parece una placa en modo BOOTSEL (no tiene INFO_UF2.TXT)")
            return 1
        unidades = [(args.unidad, leer_info_uf2(args.unidad))]
    else:
        unidades = buscar_unidades()

    if not unidades:
        port = None if args.sin_reinicio else puertos.elegir(args.puerto)
        if port:
            print(f"\nHay una placa con firmware en {port}.")
            print("Grabar la reinicia: se pierde la calibración en RAM y después hay que correr M56.")
            if not args.si and not preguntar("¿Pasarla a modo BOOTSEL y grabar?"):
                print("Cancelado.")
                return 1
            print("Pasando la placa a modo BOOTSEL…")
            reinicio_a_bootsel(port)
            unidades = esperar_unidad(15)
        if not unidades:
            print("\nNo aparece ninguna unidad RPI-RP2. Poner la placa en modo BOOTSEL a mano:\n"
                  "  1. Desenchufar el USB de la Pico.\n"
                  "  2. Mantener apretado el botón blanco BOOTSEL.\n"
                  "  3. Enchufar el USB y soltar el botón.\n"
                  "En Linux sin escritorio la unidad puede no montarse sola: montarla (por ejemplo\n"
                  "'sudo mount /dev/sdX1 /mnt') y correr 'umachining-flashear --unidad /mnt'.\n")
            unidades = esperar_unidad(args.espera, mensaje_cada=15)
        if not unidades:
            print("No apareció la unidad. No se grabó nada.")
            return 1

    if len(unidades) > 1:
        print("Hay más de una placa en modo BOOTSEL: " + ", ".join(c for c, _ in unidades))
        print("Dejar una sola conectada o elegirla con --unidad.")
        return 1
    carpeta, info_txt = unidades[0]
    board = _board_id(info_txt)
    print(f"\nUnidad: {carpeta}  ({board})")
    if "RP2350" in info_txt:
        print("Es una Pico 2 (RP2350). Este firmware está compilado para la Pico original (RP2040):\n"
              "compilarlo desde el fuente con 'board = rpipico2' en firmware/platformio.ini.")
        return 1

    # 2. grabar
    print("Grabando…")
    copiar(uf2, carpeta)
    port = esperar_puerto(20)
    if port:
        print(f"Listo: la placa se reinició con el firmware nuevo y está en {port}.")
    else:
        print("Se copió el archivo pero la placa no apareció como puerto serie en 20 s.\n"
              "Desenchufarla y volver a enchufarla (sin BOOTSEL). Si sigue sin aparecer, revisar el cable USB\n"
              "(algunos solo cargan) y los permisos del puerto (ver el README).")
        return 1
    print("\nPróximo paso: abrir la interfaz ('umachining') y apretar «Calibrar e iniciar» (M56)\n"
          "con el recorrido libre, antes de mover el eje.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
