# Cambios del 20/10/2026: filtrado de la huella del motor y anti-cogging aprendido

Se probó en la rama `prueba/calibracion-armonicos-cogging` (sale de `master` en 18d8c0b) y después se pasó a `master`.

## Resumen

Lo que quedaba en movimiento (~1,8 µm RMS a 2 mm/s) era casi todo la componente de 25 µm, el paso del motor. Se implementaron tres de las opciones propuestas:

1. **Filtrado de la tabla de calibración por armónicos** (`M56`, activo por defecto; `M56 F0` lo apaga).
2. **Compensación anticipada de la retención del motor ("anti-cogging")**: una tabla de adelanto de campo por ciclo eléctrico (`M59`), guardada en flash.
3. **Aprendizaje iterativo de esa tabla** en lazo cerrado: la primera vez con `tools/cogging_learn.py`, y después **automáticamente al final de cada `M56`** (2 pasadas, ~2 min).

Además, `M203 X<mm/s>` permite cambiar la velocidad máxima (por defecto sigue en 2 mm/s). A 4 mm/s la tabla empeoraba el error, así que **la compensación se desvanece por encima de la velocidad a la que se aprendió** (completa hasta 2 mm/s, nada a partir de 3 mm/s).

| Carril completo 0 → 255 → 0 mm a 2 mm/s | RMS ida / vuelta | máx \|error\| |
|---|---|---|
| Antes (verificación de `POS_KI = 100`, 16–20/10) | 1,78–1,83 µm | ~4,8–6,5 µm |
| Solo filtro (opción 1), `M59 S0` | 1,6 / 1,6 µm | 6,8 µm |
| **Filtro + anti-cogging aprendido (opciones 1 a 3)** | **1,0 / 1,0 µm** | 6,7 µm |

Prueba de 0 → 50 → 0 mm, misma calibración: **la componente del paso baja de 1,46 a 0,06 µm**. El RMS baja de 1,57 a 0,99 µm.

**Lo que no cambia:** el máximo de error (~6,5 µm) y los picos de salida que tocan el límite de ±81°. Vienen de otra causa.

## Archivos

| Archivo | Cambio |
|---|---|
| `src/hw_config.h` | `CALIBRATION_FILTER_MOTOR_HARMONICS`, `COGGING_ENABLED_AT_BOOT`, `COGGING_DIR_REF_MM_S`, `COGGING_FADE_END`, `COGGING_RELEARN_*`, `MAX_VELOCITY_LIMIT_MM_S` |
| `src/servo_control/actuator_calibration.cpp/.h` | `filter_motor_harmonics()` y `set_calibration_motor_filter()` |
| `src/servo_control/servo_controller.cpp/.h` | Tablas `cogging_table` y `cogging_table_dir`, y `cogging_feedforward()`, que se suma al adelanto del campo y se desvanece por encima de la velocidad de aprendizaje |
| `src/servo_control/cogging_learner.cpp/.h` | Nuevo: plegado del error y paso de Newton para el reaprendizaje en `M56`, con las respuestas G medidas |
| `src/axis_controller.cpp/.h` | `store_cogging()` y `load_cogging()` (archivo `axis_cogging.dat`, con la misma pausa de Core 1 que `store_calibration()`), y `field_angle_at_zero()` |
| `src/command_processor.cpp` | `M56 F<0\|1> A<0\|1>`, el reaprendizaje al calibrar, y los comandos nuevos `M59` y `M203` |
| `src/shared_data.h`, `src/main.cpp` | El ISR del perfil (Core 0, 2 kHz) le pasa objetivo, posición y velocidad al `CoggingLearner` |
| `maintemp.cpp` | Igual a `src/main.cpp`. **`variantes_main/main_firmware_completo.cpp` no**, porque está en solo lectura |
| `tools/cogging_learn.py` | Nuevo: aprende las tablas en lazo cerrado y grafica cada iteración |

**Cómo volver atrás:** con `M56 F0 A0` y `M59 S0` el firmware se comporta como antes. Si no, se vuelve al commit 18d8c0b.

## 1. Filtrado de la huella del motor en la calibración

La tabla encoder → motor se arma en lazo abierto, girando el campo, y supone que el rotor lo sigue de forma pareja. No es así: dentro de cada ciclo eléctrico el rotor se adelanta y se atrasa (retención en cada paso, forma de la corriente). Ese patrón queda grabado en la tabla.

Los dos patrones se separan por frecuencia:

- **Rueda de imanes:** se repite 30·m veces por vuelta.
- **Motor:** se repite 50·m veces por vuelta (50 pares de polos). 50 veces por vuelta es el ciclo eléctrico, de 100 µm de carro, y 200 veces es el paso, de 25 µm.

Al terminar la calibración, `filter_motor_harmonics()` resta de la tabla los armónicos 50·m. La fase de cada uno se toma de la posición del motor, que es donde el patrón del motor es periódico. Los armónicos que también son de la rueda (150, 300, 450…) no se pueden separar y se dejan.

El log de la calibración informa lo que sacó. En la primera prueba:

| Armónico por vuelta | Qué es | Amplitud sacada |
|---|---|---|
| 200 | paso | 0,60 µm |
| 100 | medio ciclo eléctrico | 0,34 µm |
| 400 | medio paso | 0,28 µm |
| 50 | ciclo eléctrico | 0,15 µm |

Con la tabla filtrada, en 0 → 50 → 0 mm, el RMS bajó de 1,79 a 1,58 µm y la componente del paso de 1,93 a 1,47 µm. La componente del ciclo eléctrico subió un poco (0,43 → 0,62 µm). La conmutación dio bien con el filtro (+0,1°).

## 2. Anti-cogging en el firmware

```
lead = clamp(salida_PI + G · [ tabla(fase) + s · tabla_sentido(fase) ], ±0,45 π)
campo = angulo_de_campo + lead
```

- `fase` es el ángulo eléctrico absoluto del campo, con 64 puntos por ciclo e interpolación lineal. No depende del homing.
- `s` es la velocidad del perfil dividida por la velocidad de referencia (`COGGING_DIR_REF_MM_S` = 2 mm/s), limitada a ±1. Quieto vale 0, y entra de a poco al arrancar y al frenar.
- `output` y la telemetría siguen siendo **solo la salida del PI**. Así se ve cuánto queda por corregir.

**¿Por qué dos tablas?** El lazo no responde igual al mismo patrón en los dos sentidos. En la vuelta el patrón pasa con el tiempo invertido, y la respuesta es la conjugada de la de la ida. Con una sola tabla, el aprendizaje se estancó en 1,27 µm: el promedio de ida y vuelta se anulaba, pero cada sentido quedaba con el mismo residuo con signo opuesto. Por eso hay una parte que es igual en los dos sentidos y otra que cambia de signo con el sentido.

Comando `M59` (sin argumentos informa el estado):

```
M59 S<0|1>          activar / desactivar
M59 G<g>            ganancia (1 por defecto)
M59 C               borrar las dos tablas
M59 I<i> V<°el.>    punto i de la parte igual en los dos sentidos
M59 J<i> V<°el.>    punto i de la parte que cambia con el sentido
M59 R<mm/s>         velocidad de referencia de la parte por sentido
M59 P               imprimir las tablas
M59 W / M59 L       guardar en flash / cargar de flash
```

Al arrancar, si hay tablas en flash se cargan, y se activan si `COGGING_ENABLED_AT_BOOT = true`.

## 3. Aprendizaje (`tools/cogging_learn.py`)

Cada iteración hace una ida y vuelta 0 → 50 → 0 mm a 2 mm/s con telemetría. Por sentido, se pliega el error de seguimiento sobre el ciclo eléctrico.

**Lo que no funcionó: copiar el patrón del PI a la tabla** (la versión directa de la opción 2). El RMS empeoró de 1,58 a 2,84 µm, y lo que corregía el PI subió de 20° a 48° pico a pico.

La causa: a 2 mm/s, el ciclo eléctrico cae en 20 Hz y el paso en 80 Hz. A 40–80 Hz el lazo responde con ~180° de atraso, así que la tabla empujaba para el mismo lado que la perturbación. Medido armónico por armónico: el armónico 2 respondió con +177°.

**Lo que funcionó: el método de la secante, armónico por armónico** (h = 1…8 del ciclo eléctrico, por separado en cada sentido):

1. **Iteración 0:** referencia.
2. **Iteración 1:** prueba con la tabla + 0,5 × el patrón del PI. Sirve para medir cómo responde cada armónico.
3. **Iteraciones 2 en adelante:** con las dos últimas pasadas se estima G = Δerror / Δtabla y se corrige T ← T − ganancia · error / G.

Corrida `cogging_153911_dos_tablas` (ganancia 0,8):

| Iteración | RMS | Paso (25 µm) | Ciclo (100 µm) | Lo que corrige el PI por ciclo |
|---|---|---|---|---|
| 0, referencia | 1,57 µm | 1,48 µm | 0,55 µm | 20,0° pp |
| 1, prueba | 2,17 µm | 2,36 µm | 0,36 µm | 28,6° pp |
| 2 | 1,11 µm | 0,60 µm | 0,21 µm | 4,8° pp |
| 3 | 0,99 µm | 0,07 µm | 0,17 µm | 1,8° pp |
| 4 a 5 | 0,99 µm | 0,06 µm | 0,12–0,14 µm | 1,7–1,9° pp |

Converge en tres iteraciones y queda estable. El reposo no cambia respecto de la referencia de la misma corrida. Lo que queda (~1 µm RMS) es sobre todo el resto aleatorio (0,6 µm) y la componente por vuelta del motor (0,48 µm), que este método no toca.

## 4. Después de una calibración nueva

Las tablas aprendidas dependen de la calibración. Con un `M56` nuevo y las tablas viejas (pruebas `postcal_S0` y `postcal_S1`, de 0 → 50 → 0 mm):

| | RMS ida | RMS vuelta |
|---|---|---|
| `M59 S0` (sin compensar) | 1,1 µm | 2,3 µm |
| `M59 S1` (tablas de la calibración anterior) | 1,4 µm | 1,6 µm |

En conjunto siguen ayudando, pero mucho menos. Las respuestas G, en cambio, dependen del motor y del lazo, no de la calibración. Por eso se pueden reutilizar: con ellas no hace falta la pasada de prueba, y alcanza con un paso de Newton. Desde la PC (`cogging_learn.py --g-from`, corrida `cogging_155859_reaprender`) dio **1,49 → 1,09 → 1,08 µm en dos pasadas**.

### Reaprendizaje automático en `M56`

Ahora lo hace el firmware al final de cada calibración:

1. Rearma el lazo quieto donde está el carro (como `M17`), va a 0 y espera a que se asiente.
2. Hace `COGGING_RELEARN_PASSES` = 2 idas y vueltas 0 → 50 → 0 mm a 2 mm/s, con las tablas que haya: las de flash o las de la pasada anterior. El ISR del perfil pliega el error de cada sentido durante el crucero, en 64 intervalos del ciclo eléctrico, igual que `cogging_learn.py`.
3. Entre pasadas aplica un paso de Newton (ganancia 0,8) con las respuestas G de `cogging_153911_dos_tablas`, que están escritas en `cogging_learner.cpp`. En la PC se verificó que da el mismo resultado que `cogging_learn.py` (diferencia máxima 2·10⁻⁵°).
4. Deja las tablas de la pasada con menos RMS. Con `M56 S` también las guarda en flash.

Si el error pasa de 150 µm o salta el corte por falla, frena, vuelve a las tablas de antes y lo informa. La calibración sigue valiendo igual.

Al terminar, el lazo queda **activo y quieto en 0**: ya no hace falta mandar `M17` después de `M56`, aunque mandarlo no molesta. `M56` tarda ~2,5 min en total. La respuesta incluye el RMS de cada pasada:

```
anti-cogging, RMS por pasada [um]: 1.07 1.06 -> queda la tabla de la pasada 2
ok
```

Opciones: `M56 A0` no reaprende. `M56 H` (calibrar donde está, sin buscar el tope) tampoco reaprende, salvo que se agregue `A1`, porque no se sabe si hay 50 mm libres.

Resultado con la calibración nueva, 0 → 50 → 0 mm a 2 mm/s:

| | RMS ida / vuelta | máx \|error\| |
|---|---|---|
| `M59 S0` (sin compensar) | 1,1 / 2,1 µm | 9,2 µm |
| `M59 S1` (tabla reaprendida en `M56`) | **1,1 / 1,1 µm** | 7,3 µm |

`tools/cogging_learn.py` sigue sirviendo para aprender desde cero o con más pasadas, y para medir las respuestas G de nuevo si cambia el motor, la corriente o las ganancias del lazo.

## 5. A 4 mm/s

`M203 X4` sube la velocidad máxima (el tope es `MAX_VELOCITY_LIMIT_MM_S` = 5 mm/s). Dos rondas de 0 → 50 → 0 mm, sin y con anti-cogging, con las tablas aprendidas a 2 mm/s:

| | RMS ida / vuelta | máx \|error\| | salida > 70° |
|---|---|---|---|
| Ronda 1, `M59 S0` | 3,3 / 2,9 µm | 21 µm | 35–41 % |
| Ronda 1, `M59 S1` | 4,8 / 7,7 µm | 58 µm | 37–43 % |
| Ronda 2, `M59 S0` | 4,5 / 3,9 µm | 30 µm | 36–42 % |
| Ronda 2, `M59 S1` | 5,1 / 8,1 µm | 54 µm | 37–43 % |
| `M59 S1` con el desvanecimiento | 4,9 / 3,9 µm | 33 µm | 41–45 % |

Se ven dos cosas:

- **A 4 mm/s el motor se queda sin torque.** La salida media es ±68,5° contra el límite de 81°, y pasa el 35–45 % del tiempo arriba de 70°. El error es 3–4 veces el de 2 mm/s, con o sin tabla. Para ir más rápido hace falta más margen de torque (corriente), no calibración.
- **La tabla aprendida a 2 mm/s empeora las cosas a 4 mm/s**, como se temía: el atraso del lazo cambia con la frecuencia. Por eso `cogging_feedforward()` multiplica la compensación por un factor que vale 1 hasta `COGGING_DIR_REF_MM_S` y baja lineal hasta 0 en `COGGING_FADE_END` × esa velocidad (3 mm/s). Con eso, a 4 mm/s queda igual que sin tabla.

## Limitaciones y pendientes

- **La compensación solo ayuda cerca de 2 mm/s.** Arriba de 3 mm/s se apaga sola. A 1 mm/s no se probó: ahí está completa, y podría no ayudar. Si se trabaja a otra velocidad, habría que aprender tablas y respuestas G para esa velocidad.
- **¿Movimiento real o error de medición?** Sigue sin saberse si los 25 µm son movimiento real del carro o un error de la tabla de calibración. La opción 1 saca de la medición la parte que viene de la tabla, y las opciones 2 y 3 corrigen la parte que es movimiento. Las dos juntas bajan el error medido por el encoder, pero solo un sensor externo rápido (ensayo E6) confirma que el carro se mueve mejor.
- **Máximo y saturación.** El máximo de error (~6,5 µm) y los toques del límite de ±81° no cambiaron.
- **Opción 4 sin hacer.** No se hizo la verificación de conmutación en una vuelta completa.

## Datos

Todo está en `mediciones/2026-10-20/`:

- `cogging_152532_sin_filtro` y `cogging_152712_con_filtro`: opción 1, y el intento de copiar el patrón del PI.
- `cogging_153207_secante`: secante con una sola tabla, que se estancó en 1,27 µm.
- `cogging_153911_dos_tablas`: el aprendizaje final.
- `cogging_155859_reaprender`: el reaprendizaje rápido después de una calibración nueva.
- `lazo_cerrado_20261020_154542_filtro_cogging` y `_155024_filtro_solo`: el carril completo.
- `lazo_cerrado_20261020_1556*_postcal_S0` y `_1557*_postcal_S1`: las tablas viejas con una calibración nueva.
- `lazo_cerrado_*_4mm_r1_S*`, `_4mm_r2_S*` y `_4mm_fade_S1`: las pruebas a 4 mm/s.
- `lazo_cerrado_*_reaprendido_2mm_S*`: la verificación del reaprendizaje en `M56`.

Cada carpeta `cogging_*` tiene `cogging_iteraciones.png`. Para regenerarlo: `python3 tools/cogging_learn.py --from-dir <carpeta>`.
