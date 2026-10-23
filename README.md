# μMachining

**Español** · [English](README.en.md)

Firmware de control en lazo cerrado para un eje lineal de micromaquinado, sobre un RP2040.

Un motor paso a paso mueve un carro con un husillo de bolas. Un encoder magnético sobre el eje del motor cierra el lazo, y el control es por campo (FOC), no por pasos. El firmware está adaptado de [MicroManipulatorStepper](docs/README_ORIGEN_DEL_CODIGO.md).

**Autores:** Francisco Maresca · Tiago Meilan

## Características

- Lazo en cascada posición → velocidad → campo, a ~20 kHz en un núcleo dedicado
- Calibración automática del encoder y de la conmutación, con guardado en flash
- Realimentación anticipada de velocidad desde el perfil de movimiento
- Homing contra el tope mecánico, con verificación de la conmutación
- Corte de seguridad por error de seguimiento
- Telemetría binaria a 1 kHz y herramientas en Python para pruebas, barridos de ganancias y análisis de errores

## Rendimiento

Recorrido completo (0–255 mm) a 2 mm/s:

| | |
|---|---|
| Error de seguimiento en movimiento | 1,2–1,9 µm RMS (máx. 4–8 µm) |
| Error en reposo | 0,002 µm (desvío estándar, `POS_KI = 100`) |
| Resolución del encoder | 0,08 nm por cuenta |

> Son errores de seguimiento: la posición medida por el encoder contra la pedida. La exactitud absoluta del carro todavía no se midió con una referencia externa.

## Hardware

| | |
|---|---|
| Placa | Raspberry Pi Pico (RP2040) |
| Motor | Paso a paso de 1,8° (50 pares de polos) |
| Driver | TB6612 |
| Transmisión | Husillo de bolas de 5 mm por vuelta |
| Encoder | MT6835 de 21 bits frente a una rueda de 60 imanes |

## Inicio rápido

```bash
# dependencias de las herramientas de PC
pip install numpy matplotlib pyserial PyQt5 pyqtgraph

# compilar y cargar (cerrar antes el monitor serie)
pio run -t upload
```

Por serie a 921 600 baudios:

```
M56        homing + calibración
M17        habilitar el lazo cerrado
G1 X10     ir a X = 10 mm
M114       posición actual
```

O con la interfaz gráfica ([guía de uso](docs/GUIA_INTERFAZ.md)):

```bash
python3 tools/interfaz.py
```

O con el script:

```bash
python3 tools/send_gcode.py "M56" "M17" "G1 X10" "M114" --timeout 600
```

## Comandos

| Comando | Descripción |
|---|---|
| `G28` | Homing contra el tope |
| `M56 [S]` | Homing + calibración (`S`: guardar en flash) |
| `M17` / `M18` | Habilitar / apagar el motor |
| `G1 X<mm> [F<mm/s>]` | Mover (0–255 mm, hasta 2 mm/s) |
| `G90` / `G91` | Modo absoluto / relativo |
| `M114` / `M119` | Posición / estado completo |
| `M55 A B C D F` | Ganancias del PID en caliente |
| `M58 V<0\|1> F<0\|1>` | Velocidad con la tabla de calibración / realimentación anticipada |
| `M120 S1 P<n>` | Telemetría binaria |
| `M57 X<mm>` / `M121` | Diagnóstico de conmutación / traza del último corte por falla |

## Herramientas

| Script | Descripción |
|---|---|
| `tools/interfaz.py` | Interfaz gráfica: control remoto y telemetría en vivo ([guía](docs/GUIA_INTERFAZ.md)) |
| `tools/send_gcode.py` | Mandar comandos en secuencia |
| `tools/closed_loop_test.py` | Idas y vueltas con telemetría y gráficos de todo el recorrido |
| `tools/pid_sweep.py` | Barrido de ganancias en caliente, con el error en movimiento y en reposo |
| `tools/ki_sweep.py` | Barrido de una constante de `hw_config.h`, recompilando para cada valor |
| `tools/analisis_errores.py` | Descomposición del error en sus componentes |
| `tools/encoder_sweep.py` | Caracterización del encoder |

Los resultados se guardan en `mediciones/AAAA-MM-DD/`.

## Estructura

```
src/            firmware (hw_config.h: parámetros del montaje y del PID)
tools/          scripts de PC en Python
docs/           documentación, reportes y registro de cambios
mediciones/     datos y gráficos de cada prueba, por día
simulaciones/   modelo del campo magnético de la rueda
```

## Notas

- **Correr `M56` en cada arranque, no `G28`.** La calibración restaurada desde la flash puede dejar la conmutación corrida.
- **`POS_KI = 100`.** Deja el carro quieto en reposo (0,002 µm) sin cambiar el movimiento; por encima de ~200–250 la integral oscila contra la fricción estática ([elección](docs/ELECCION_POS_KI.md)).
- **Forma de trabajo:** `master` es siempre una versión estable y etiquetada; cada prueba va en una rama `prueba/<nombre>`.

## Documentación

- [Arquitectura del firmware](docs/DOCUMENTACION_ARQUITECTURA.md)
- [Encoder y calibración](docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md)
- [Reporte de la prueba de lazo cerrado](docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md)
- [Método de análisis de errores y plan de caracterización](docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md)
- [Elección de POS_KI](docs/ELECCION_POS_KI.md)
- [Registro de cambios](docs/CAMBIOS_2026-10-16.md) (y los `CAMBIOS_*.md` anteriores)

## Créditos

Basado en el firmware `MotionControllerRP` de MicroManipulatorStepper (licencia MIT). Ver [qué se reutilizó y qué se reescribió](docs/README_ORIGEN_DEL_CODIGO.md).
