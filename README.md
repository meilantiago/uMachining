# Firmware de eje único (X) para micromaquinado

Proyecto PlatformIO para un RP2040 que controla un eje lineal: husillo de bolas de **5 mm por vuelta**, motor paso a paso de 1,8° (50 pares de polos), driver TB6612 y encoder magnético MT6835 frente a una rueda de **60 imanes** (30 períodos de encoder por vuelta). Está adaptado de MicroManipulatorStepper; el origen de cada archivo está en [docs/README_ORIGEN_DEL_CODIGO.md](docs/README_ORIGEN_DEL_CODIGO.md).

## Estructura de la carpeta

```
single_axis_firmware/
├── README.md              este archivo
├── platformio.ini         configuración de PlatformIO (no mover)
├── src/                   código que se compila y se carga en la placa
│   ├── main.cpp           el main que se compila (una de las variantes de abajo)
│   └── hw_config.h        parámetros del montaje (husillo, imanes, corriente, PID, pines)
├── include/               vacío, lo pide PlatformIO
├── variantes_main/        los distintos main.cpp, para copiar a src/
│   ├── main_firmware_completo.cpp   firmware real (G1, G28, M56, M17...)
│   └── main_barrido_encoder.cpp     firmware de barrido (solo G56, para encoder_sweep.py)
├── tools/                 scripts de PC (Python)
├── docs/                  documentación, historial y registro de sesiones
├── mediciones/            una carpeta por día: AAAA-MM-DD/
├── simulaciones/          notebook del campo magnético (magpylib)
└── respaldos/             copias viejas, para volver atrás si hace falta
```

Fuera de esta carpeta, en `../checkpoints/`, se guardan copias congeladas del proyecto (ver más abajo).

## Cambiar de firmware

PlatformIO compila solo lo que está en `src/`, así que para cambiar de firmware se copia la variante a `src/main.cpp` y se carga. Hay que cerrar antes el monitor serie.

```bash
# firmware real
cp variantes_main/main_firmware_completo.cpp src/main.cpp
pio run -t upload

# firmware de barrido del encoder
cp variantes_main/main_barrido_encoder.cpp src/main.cpp
pio run -t upload
```

Si se modifica `src/main.cpp`, hay que copiarlo de vuelta a `variantes_main/` para no perder el cambio la próxima vez que se cambie de firmware. Los demás archivos de `src/` son comunes a las dos variantes.

## Herramientas

Todas necesitan el puerto serie libre (cerrar el monitor de PlatformIO).

| Comando | Firmware | Qué hace |
|---|---|---|
| `python3 tools/encoder_sweep.py` | barrido | Barrido del encoder. Guarda CSV y PNG en `mediciones/AAAA-MM-DD/` |
| `python3 tools/plot_periods.py mediciones/AAAA-MM-DD/archivo.csv` | ninguno | Análisis por sectores de la rueda de un CSV ya guardado |
| `python3 tools/capture_calibration.py --out mediciones/AAAA-MM-DD/calib --timeout 600` | real | Corre `M56 P` y guarda los datos de la calibración |
| `python3 tools/send_gcode.py "G28" "M17" "G1 X10" "M119"` | real | Manda comandos en secuencia |
| `python3 tools/analisis_errores.py mediciones/AAAA-MM-DD/lazo_cerrado_X.npz` | ninguno | Figuras de análisis de errores para el reporte (en `reporte/`, junto al .npz) |
| `python3 tools/ki_sweep.py` | real | Barrido de `POS_KI` (u otra constante): compila, carga, calibra, y mide el error en reposo en X20 y X100 durante 10 s. Restaura `hw_config.h` al final |
| `python3 tools/pid_sweep.py` | real | Barrido de ganancias del PID (con `M55`, sin recompilar): error a lo largo del eje y por componente para cada configuración. `--list` muestra las configuraciones sin mover nada |
| `python3 tools/closed_loop_test.py` | real | Idas y vueltas en lazo cerrado con telemetría a 1 kHz y gráficos de todo el carril (ver docs/CAMBIOS_2026-10-16.md) |

## Uso del firmware real

Por serie a 921600 baudios:

```
G28            homing contra el tope mecánico + verificación de la conmutación
M56            homing + calibración (M56 S la guarda en flash)
G28 H / M56 H  igual, pero toma como home el lugar actual, sin tope (solo motor desacoplado)
M17            habilitar el motor en lazo cerrado (exige G28 o M56; queda quieto donde está)
G1 X10         ir a X = 10 mm (absoluto, 0 = tope, rango 0-255 mm)
G91 / G90      modo relativo / absoluto
M114           posición
M119           estado completo (incluye el último corte por falla)
M58 V1 F1      velocidad con la tabla / realimentación anticipada (por defecto las dos en 1; V0 F0 = como antes del 16/10)
M120 S1 P2     telemetría binaria a 1 kHz (la lee tools/closed_loop_test.py); M120 S0 la apaga
M121           traza del lazo de los ~2 s previos al último corte
M57 X250       diagnóstico: barrido en lazo abierto con el desfase de conmutación (ver docs/CAMBIOS_2026-10-13.md)
M18            apagar el motor
```

Si el error de seguimiento pasa 1 mm durante 0,5 s, el firmware apaga el motor (`fault=1` en `M119`). En ese caso `G1` responde `ok` pero no mueve: hay que mandar `M17` y repetir el `G1`.

## Documentación

| Archivo | Contenido |
|---|---|
| [docs/DOCUMENTACION_ARQUITECTURA.md](docs/DOCUMENTACION_ARQUITECTURA.md) | Diseño general del firmware |
| [docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md](docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md) | Cómo se caracteriza el encoder y cómo funciona la calibración |
| [docs/HISTORIAL_DE_CAMBIOS_FIRMWARE.md](docs/HISTORIAL_DE_CAMBIOS_FIRMWARE.md) | Historial de cambios hasta el 29/09 |
| [docs/SESION_2026-09-29.md](docs/SESION_2026-09-29.md) | Sesión del 29/09: reubicación del encoder |
| [docs/CAMBIOS_2026-10-02.md](docs/CAMBIOS_2026-10-02.md) | Cambios del 02/10: 60 imanes, husillo de 5 mm, corte del lazo, homing |
| [docs/CAMBIOS_2026-10-09.md](docs/CAMBIOS_2026-10-09.md) | Cambios del 09/10: conmutación desfasada tras el homing, cómo probar |
| [docs/SESION_2026-10-13.md](docs/SESION_2026-10-13.md) | Sesión del 13/10: diagnóstico de la vuelta trabada y de la vibración en reposo, con mediciones |
| [docs/CAMBIOS_2026-10-13.md](docs/CAMBIOS_2026-10-13.md) | Cambios del 13/10: conmutación corrida en cada vuelta (vuelta al home trabada), `M57` |
| [docs/CAMBIOS_2026-10-16.md](docs/CAMBIOS_2026-10-16.md) | 16/10: `M56 S` colgaba la placa, telemetría `M120` completa, prueba de idas y vueltas |
| [docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md](docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md) | Reporte de la prueba de lazo cerrado: qué es cada variable, tipos de error, de dónde sale cada uno y cómo reducirlos |
| [docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md](docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md) | Cómo se separa cada contribución al error (plegado y experimentos), qué vería un comparador, y plan de ensayos para el error instrumental y la precisión |
| [docs/PLAN_DE_PRUEBAS.md](docs/PLAN_DE_PRUEBAS.md) | Plan de pruebas |
| [docs/README_ORIGEN_DEL_CODIGO.md](docs/README_ORIGEN_DEL_CODIGO.md) | README original: de dónde sale cada archivo |

**Los documentos anteriores al 02/10 usan las rutas viejas** y algunos datos que después cambiaron (por ejemplo, el husillo de 2 mm). Equivalencias:

| Ruta vieja (en los documentos) | Ruta actual |
|---|---|
| `main_firmware_completo.cpp.bak` | `variantes_main/main_firmware_completo.cpp` |
| `main_barrido_encoder.cpp.bak` | `variantes_main/main_barrido_encoder.cpp` |
| `main_firmware_completo.cpp.bak.old` | `respaldos/main_firmware_completo.cpp.bak.old` |
| `respaldo_2026-10-02_antes_de_*` | `respaldos/2026-10-02_antes_de_*` |
| `mediciones/29_9/`, `mediciones/2_10/` | `mediciones/2026-09-29/`, `mediciones/2026-10-02/` |
| `SIMULACIONES_CLEAN.ipynb` | `simulaciones/SIMULACIONES_CLEAN.ipynb` |
| `*.md` en la raíz | `docs/*.md` |

## Checkpoints

Un checkpoint es una copia congelada del proyecto en `../checkpoints/`. Se trabaja siempre en esta carpeta (`single_axis_firmware/`), y el checkpoint queda como referencia para volver a una versión que andaba.

- Los checkpoints son de solo lectura, para no modificarlos por error.
- No incluyen `mediciones/`, que es pesada y no cambia, ni `.pio/`, que PlatformIO regenera al compilar.

Para volver a un checkpoint, copiar `src/` (y si hace falta `tools/` y `variantes_main/`) desde la copia a esta carpeta.

| Checkpoint | Estado |
|---|---|
| `../checkpoints/single_axis_firmware_2026-10-02/` | Husillo de 5 mm, 60 imanes, homing con confirmación del tope, `M17` rearma quieto. Calibra, homea y mueve en lazo cerrado. |
