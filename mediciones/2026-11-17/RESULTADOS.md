# Pruebas del 17/11/2026: cople nuevo y re-sintonía del PID

Firmware en la placa: el que estaba cargado (no se regrabó). Corriente 0,6. Anti-cogging encendido con las tablas que tenía la placa. Todas las ganancias se cambiaron en caliente con `M55`.

## Problema

Desde que se cambió el cople por uno más ajustado, `M56` dejaba el carro a mitad del stage, el movimiento siguiente "volvía a 0" y después oscilaba.

## 1. Estado de la placa al empezar (`traza_corte_previo.csv`)

- Estaba en falla. El último corte fue con el objetivo en 1 mm y el carro clavado en 0,636 mm durante más de 1 s, con la salida saturada en +81°. Después el carro corrió hacia atrás con la salida todavía en +81°.
- El cero anterior no era el tope real. Desde −1,85 mm (según la referencia vieja), el homing de `M56 A0` recorrió 2,28 vueltas (11,4 mm) hasta el tope.

## 2. Calibración y carga en lazo abierto (`m56_a0.log`, `m57_x5.log`)

- `M56 A0`: desfase de conmutación −1,6°, calibración bien. Hubo un "patinazo del rotor" durante el homing.
- `M57 X5`: atraso del rotor de **+46,8°** con patrones cada ~1,25 mm. El 06/11 era **+30,2 ± 3,7°**. La carga media subió ~40 % (sin 30° → sin 47°).

## 3. Puntos duros una vez por vuelta (`barrido_pid_171511/`, `relevo_6_22_1mms.npz`)

- Barrido estándar de 13 configuraciones, 2 → 22 mm a 2 mm/s: **las 13 se cortaron en 4,93 ± 0,007 mm**, con la salida en 81°. El fallo no depende de las ganancias.
- Relevo a 1 mm/s: hay un punto duro cada 5 mm, en **~4,9, 9,9, 14,9 y 19,9 mm**, o sea una vez por vuelta del motor. En cada uno la salida satura en 81° durante ~1 mm y el error sube a 90–125 µm. Entre puntos duros la salida es de 45–59° y el error ≤ 6 µm.
- A 0,5 mm/s el punto de 4,9 mm se pasa con 28–48 µm de error.
- Una carga que vuelve una vez por vuelta es la firma de un cople forzado o desalineado (o de excentricidad del eje o del husillo). **Ningún PID lo arregla:** la salida ya está en el tope.

## 4. PID en la ventana limpia 11 → 14,2 mm, 2 mm/s (`barrido_pid_171851/`, `_172320/`, `_172800/`)

| Configuración | RMS [µm] | Máx. [µm] | Máx. salida | Asentamiento | Reposo largo (5 s) |
|---|---|---|---|---|---|
| Base: 60 / 100 / 0,12 / 54 / 4 ms | 1,54–1,58 | 4,9–5,5 | 73–75° | 73–85 nm | 3,6–3,9 nm |
| **`VEL_KI` 40, `POS_KI` 50** | **1,43–1,45** | 4,6–6,7 | 71–73° | **11–13 nm** | **2,6–3,2 nm** |
| `VEL_KI` 40, `POS_KI` 0 | 1,48 | 6,4 | 72° | 21 nm | 1,55 nm |
| `VEL_KP` 0,14, `VEL_KI` 40, `POS_KI` 50 | 1,41 | 4,5 | 73° | 14 nm | 3,0 nm |

- `VEL_KP`: 0,06 da 3,0 µm y 0,09 da 1,8 µm; 0,15 da lo mismo que 0,12. No apareció la oscilación de ~214 Hz con ninguno.
- `VEL_KI`: 108 satura (5,4 µm). 27 da el mismo RMS que 40, pero se asienta peor (150 nm).
- `POS_KP` 40–120: no cambia el movimiento.
- `VEL_LOWPASS_TC`: 4 ms sigue siendo lo mejor; 8 ms da 3,5 µm.

Primera elección: `POS_KI` 50, `VEL_KI` 40. **Quedó descartada en la sección 5:** el reposo de esas rondas era de solo 1 s y no mostraba la caza de ~0,5 Hz.

## 5. Caza en reposo después de mover (`caza_reposo_11mm.npz`, `barrido_pid_173858/`, `reposo_largo_12p5_b.npz`)

En la interfaz, quieto en 11 mm después de un `G1`, el carro oscilaba en onda cuadrada de ~0,5 Hz y ±0,3 µm, con la salida en apenas 13 % (−10,5°). No es falta de torque: es stick-slip. La integral acumula contra la fricción estática hasta que el carro se despega y salta al otro lado. Después de `M17` (integrales en cero) el reposo es de 0,02 µm; la caza aparece recién después de mover. Ya estaba en todos los barridos, a 11 mm (0,3–1,1 µm p-p) y a 14,2 mm (0,1–0,4 µm).

Protocolo de `caza_reposo.py`: `M55`, ir a 12 mm y volver a 11 mm a 1 mm/s, y medir 6 s quieto.

| `M55` | Reposo p-p a 11 mm | Máx. error en movimiento |
|---|---|---|
| A60 B100 C0.12 D54 (base) | 0,74 µm | 4,3 µm |
| A60 B50 C0.12 D40 | 0,63–0,69 µm | 5,3 µm |
| A60 B0 C0.12 D40 | 0,21 µm | 4,9 µm |
| A90 B0 C0.12 D40 | 0,12 µm | 4,5 µm |
| **A120 B0 C0.12 D40** | **0,06 µm** | 4,3 µm |
| A120 B50 C0.12 D40 | 0,17 µm | 4,8 µm |
| A120 B0 C0.12 D54 | 0,03 µm | 3,8 µm |
| A150 B0 C0.12 D40 | 0,03 µm | 3,7 µm |

Validación a 2 mm/s, 11 → 14,2 mm, 3 ciclos, 4 s de reposo en cada punta (`barrido_pid_173858/`):

| Configuración | RMS | Máx. salida | Reposo p-p a 11 mm | Reposo p-p a 14,2 mm |
|---|---|---|---|---|
| Base 60 / 100 / 0,12 / 54 | 1,59–1,60 µm | 74–75° | 0,69–0,73 µm | 0,25–0,33 µm |
| 60 / 50 / 0,12 / 40 | 1,49 µm | 72° | 0,63–0,70 µm | 0,21 µm |
| **120 / 0 / 0,12 / 40** | **1,59 µm** | **76°** | **0,26 µm** | **0,03–0,04 µm** |
| 120 / 0 / 0,12 / 54 | 1,65 µm | 79° | 0,15–0,18 µm | 0,03 µm |
| 150 / 0 / 0,12 / 54 | 1,72 µm | 80,5° | 0,10 µm | 0,02 µm |

- Sin `POS_KI` no queda error permanente: el error medio en reposo es ≤ 0,016 µm (lo anula la integral de `VEL_KI`).
- Reposo largo en 12,5 mm después de `M17` (5 s a 2 kHz): base 3,0 nm, 120 / 0 / 0,12 / 40 2,2 nm. No aparece la oscilación de ~214 Hz. En una repetición hubo un salto aislado de 120 nm p-p (desvío 4 nm).
- Las de mejor reposo usan casi todo el torque en movimiento (79–80,5° de 81°). Con los puntos duros del cople no conviene quedarse sin margen.

**Elegido:** `POS_KP` 120, `POS_KI` 0, `VEL_KP` 0,12, `VEL_KI` 40, `VEL_LOWPASS_TC` 0,004 (`M55 A120 B0 C0.12 D40 F0.004`). Quedó cargado en caliente y en `hw_config.h` (rama `Firmware+Senoide`, sin commitear ni grabar). La caza que queda a 11 mm (0,26 µm) depende de la fricción de ese punto; es la misma mecánica que los puntos duros.

## Pendiente

- Alinear o aflojar el cople, y repetir el relevo a 1 mm/s (`explore.py`, en esta carpeta) para comprobar que desaparecen los puntos de 4,9 / 9,9 / 14,9 / 19,9 mm.
- Si no alcanza, subir `MOTOR_MAX_CURRENT_FACTOR` (0,6 → 0,8) y bajar `VEL_KP` en la misma proporción, vigilando la temperatura.
- Con la mecánica así, el reaprendizaje anti-cogging de `M56` (0 → 50 mm a 2 mm/s) se va a cortar siempre en un punto duro. Usar `M56 A0` hasta arreglarlo.
- Repetir el barrido en todo el eje cuando pase sin cortes.

## 6. Pasos chicos: 10 nm lentos y 1 µm con sobrepaso (`escalones.py`, `escalones_12p5*.npz`)

Con 120 / 0 / 0,12 / 40, a 12,5 mm:

| Escalón | Al 90 % | Sobrepaso |
|---|---|---|
| 10 nm | 0,6–1,1 s | 40–52 % |
| 100 nm | — | 89–106 % |
| 1 µm | 50–80 ms | 25–61 % |
| 10 µm | 65 ms | 5 % |

Es stick-slip. La proporcional da ~3,9° de salida por µm de error (rigidez `VEL_KP`·`POS_KP` + `VEL_KI` = 54 rad/rad), y despegar el carro pide ~8–10° más. En los pasos de menos de ~2 µm, quien despega el carro es la integral:

- **100 nm:** el carro queda pegado ~300 ms mientras la salida sube de 13 a 21°. Después salta a 200 nm.
- **1 µm:** pegado 30 ms, se despega con 24° y llega a 1,6 µm.

Si la integral es lenta, el paso de 10 nm tarda; si es rápida, el de 1 µm se pasa. Con el cople viejo (06/11), los pasos de 10 nm ya tardaban 170–580 ms y se pasaban 45–150 %.

Con `VEL_KI` 80–110 mejoran los dos extremos: 10 nm en 0,2–0,5 s, y 100 nm con 40–50 % de sobrepaso. Pero a 2 mm/s dan 5–7 µm RMS con la salida saturada (`barrido_pid_180922/`).

**Firmware:** `VEL_KI` cambia según la velocidad del perfil. Por debajo de `VEL_KI_HOLD_SPEED_MM_S` (0,5 mm/s) se usa `VEL_KI_HOLD` (80); por encima, `VEL_KI` (40).

- `PIDController::set_ki` cambia la ganancia sin salto, porque la integral se guarda ya multiplicada por kI.
- `M55 H<ki quieto> V<mm/s>` los cambia en caliente (`H0` desactiva el cambio); `D` sigue siendo la de movimiento.
- Un paso de hasta ~50 µm va entero con `VEL_KI_HOLD`.

Grabado el 17/11 a las 18:1x (SHA-256 `2752ce7c…`, incluye `M57 … R1` de la otra sesión) y comparado con el mismo firmware y la misma fricción:

| Escalón | `H0` (40 siempre) | `H80` | `H110` |
|---|---|---|---|
| 10 nm, al 90 % | 117–278 ms | 105–161 ms | 10–117 ms, con saltos de 5–6 nm |
| 100 nm, asentado | 279–287 ms | 137–152 ms | 105–113 ms |
| −1 µm, asentado | 214 ms | 120 ms | 150 ms, con saltos |
| A 2 mm/s, 13 → 16,5 mm (RMS) | 1,40 µm | 1,41 µm | — |
| Reposo p-p después de mover | 0,03–0,32 µm | 0,02–0,08 µm | — |

El +1 µm sigue despegándose de golpe (500–800 % de sobrepaso). En ese punto la salida llega a 38–48° antes de que el carro se suelte, contra 24–26° antes de grabar: la fricción estática subió.

## 7. Homing en lazo abierto y el cople que patina

- Después de grabar, `M56 A0` falló dos veces con 0,6 en lazo abierto: homing "en falso" a 0,17 y 0,10 vueltas, y calibración con "9,34" y "4,58 periodos por vuelta". Se agregó `OPEN_LOOP_CURRENT` = 0,8 para el homing y los barridos de calibración (el lazo cerrado sigue en 0,6). Con eso calibró: 30,003 periodos por vuelta, conmutación −1,2°. El homing contó 16,9 vueltas de campo hasta el tope.
- **Los puntos duros se corrieron:** antes en ~4,9 / 9,9 / 14,9 / 19,9 mm, ahora en **~12,1 y ~17,0 mm** (`relevo_post_flash_05mms.npz`), unos 2,2 mm (~160° de giro). La relación angular entre el motor y el husillo cambió: **el cople patinó sobre uno de los ejes**.
- Eso explica también lo del principio del día: el cero estaba corrido ~13 mm y el encoder marcaba −1,85 mm, más allá del tope. Si el cople patina, el encoder del motor cuenta vueltas que el husillo no da.

## Pendiente (actualizado)

- **Revisar el cople:** prisioneros sobre el plano del eje o cople de abrazadera, y la alineación. Hacer una marca con fibra sobre cople y ejes para ver si se corre. Hasta arreglarlo, cualquier referencia (homing, tablas) puede perderse.
- Después, repetir el relevo a 0,5 mm/s (`explore.py`), los escalones (`escalones.py`) y el barrido en todo el eje. Con el cople firme y alineado, probablemente se pueda volver a `OPEN_LOOP_CURRENT` 0,6.
- `M56` con anti-cogging: no correrlo hasta arreglar el cople (se corta en los puntos duros). Usar `M56 A0`.

## 8. Corriente del lazo cerrado 0,6 → 0,8

**Por qué el lazo abierto pasa y el cerrado se traba en el punto duro.** El lazo cerrado pone el campo en "posición según la tabla + salida", con la salida limitada a ±81°. En lazo abierto el rotor se atrasa lo que haga falta. El `M57` sobre el punto duro de 4,86–4,93 mm, medido en coordenadas de la tabla, dio atrasos de 90–107°: más que el tope del lazo cerrado. La tabla se arma en lazo abierto suponiendo un atraso parejo y en el punto duro tiene error, así que 81° "de tabla" son menos ángulo real. En lazo abierto el rotor también se atrasa (~28 µm a 100°), pero no aparece como error.

**Cambio:** `MOTOR_MAX_CURRENT_FACTOR` 0,8. Para que la ganancia del lazo sea la misma que con 0,6, las ganancias del lazo de velocidad se escalaron ×0,75: `VEL_KP` 0,09, `VEL_KI` 30, `VEL_KI_HOLD` 60. `POS_KP` queda en 120 y `POS_KI` en 0. Grabado el 17/11 (SHA-256 `e7dcb53d…`), `M56 A0`: homing de 45,43 vueltas desde 226 mm, 30,002 periodos por vuelta, conmutación −1,7°.

- **2 → 50 → 2 mm a 2 mm/s**, cruzando ~10 puntos duros (`recorrido_2_50_2mms_corriente08.npz`): error máximo **6–7 µm** (con 0,6 eran ~90 µm cada 5 mm), RMS 1,27 / 1,53 µm. La salida toca el tope un instante en los puntos duros (0,04–0,4 % del tiempo), pero ya no acumula error.
- **Reposo largo a 30 mm** (`reposo_largo_corriente08*.npz`): 3–5 nm de desvío, con picos de 11 nm entre corridas. Aparece un modo de ~130 Hz que con 0,6 era más chico (1,7–4 nm a 12,5 mm). `VEL_KP` 0,07–0,11 no lo mejora de forma consistente: 0,07 deriva a 2–7 Hz y 0,11 empeora.
- **Escalones a 30 mm** (`escalones_30_corriente08.npz`, `VEL_KI_HOLD` 45 / 60 / 80):
  - 10 µm: perfecto (0–2 % de sobrepaso, 65 ms).
  - 1 µm: el carro se despega con apenas ±9…24° y salta 3–4 µm durante 10–20 ms, pero en ~100 ms queda a menos de 0,1 µm. Ese salto tan rápido con tan poca fuerza parece energía elástica acumulada (el cople torsionado) que se suelta al despegar.
  - 10 nm y 100 nm: igual de ruidosos que antes.
  - Entre 45, 60 y 80 no hay diferencias claras.
