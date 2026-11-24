"""Dónde se guardan las mediciones de la interfaz.

Por defecto en ``~/uMachining/mediciones/AAAA-MM-DD/``. Se cambia con la variable de entorno
``UMACHINING_DATOS`` o con ``umachining --datos <carpeta>``.
"""
import os

VARIABLE = "UMACHINING_DATOS"
POR_DEFECTO = os.path.join("~", "uMachining", "mediciones")


def mediciones():
    """Carpeta raíz de las mediciones (se crea si no existe)."""
    carpeta = os.path.abspath(os.path.expanduser(os.environ.get(VARIABLE) or POR_DEFECTO))
    os.makedirs(carpeta, exist_ok=True)
    return carpeta


def del_dia(fecha):
    """Carpeta ``AAAA-MM-DD`` de un ``datetime``, creada si hace falta."""
    carpeta = os.path.join(mediciones(), fecha.strftime("%Y-%m-%d"))
    os.makedirs(carpeta, exist_ok=True)
    return carpeta


def corta(path):
    """Ruta para mostrar: relativa a la carpeta de mediciones si está adentro, si no completa."""
    raiz = mediciones()
    try:
        rel = os.path.relpath(path, raiz)
    except ValueError:            # Windows: otra unidad
        return path
    return path if rel.startswith("..") else os.path.join("mediciones", rel)
