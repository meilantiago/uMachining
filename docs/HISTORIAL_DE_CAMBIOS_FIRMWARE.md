# Historial de depuración y cambios del firmware (eje único)

Este documento registra la sesión de depuración del firmware de eje único (RP2040 + driver TB6612 + encoder MT6835 frente a una rueda magnética multipolar, husillo de 2 mm/vuelta). Explica qué fallaba, cómo se llegó a cada causa, qué se cambió en cada archivo y por qué, y qué queda pendiente.

Complementa a [DOCUMENTACION_ARQUITECTURA.md](DOCUMENTACION_ARQUITECTURA.md), que describe el diseño general.

---

## 1. Punto de partida

**Síntoma reportado:** cada componente andaba por separado (el motor giraba con un script propio y el encoder leía el ángulo crudo), pero el firmware completo "no hacía nada". La calibración (`M56`) imprimía esto:

```
, ,
, ,
E)Magnetic field too weak for 2048 of 1024 measurements
D)Optimizing lookup table finished: rms_error=
W)Fitting error of lookup table 'encoder_raw_angle -> motor_pos' unusually high (rms_error =  deg).Calibration might be invalid.
I)finished
I)AxisController: calibracion OK
```

El log ya mostraba tres problemas:

- Los valores con `%f` salían vacíos (`, ,` y `rms_error=` sin número).
- El 100 % de las mediciones venían con campo débil. El mensaje decía "2048 de 1024" porque contaba mal el total.
- Pese a los errores, terminaba con `calibracion OK`: el firmware no abortaba ante datos inválidos.

---

## 2. Estado final (resumen)

| Qué | Estado |
|---|---|
| Motor en lazo abierto | ✅ Funciona (se corrigió el pin PWM de la bobina B). |
| Homing (`G28`) | ✅ Encuentra el tope de forma repetible. Ya no entra en bucle infinito si no lo encuentra. |
| Calibración (`M56`) | ✅ Pasa con error de tabla ~0.13–0.15° RMS. Mide 29.94 ≈ 30 períodos de encoder por vuelta, que coincide con la geometría de la rueda. Se reintenta sola hasta 3 veces si sale mala. |
| Lazo cerrado (`M17` + `G1`) | ⚠️ **Parcial.** Se mueve y posiciona en el primer tramo (probado: `G1 X0.5` → 0.535 mm). **Se traba en ~1.17 mm** (≈212° de la vuelta del motor) por una zona de la rueda con campo muy débil, donde el encoder pierde el seguimiento. |
| Guardado en flash (`M56 S`) | ⚠️ Funciona, pero no está verificado que la tabla guardada siga siendo válida después de un reinicio. Por ahora conviene correr `M56` en cada arranque. |

**Causa de fondo del problema pendiente:** es física. En una parte de cada período, y sobre todo en la zona de ~212–219° de la vuelta del motor, el campo de la rueda es tan débil o deformado que el MT6835 reporta ángulos erróneos. Ningún filtro en software los distingue al 100 % de un movimiento real. Ver la sección 7.

---

## 3. Cronología del diagnóstico

Se incluyen también las hipótesis que resultaron equivocadas: explican por qué ciertos cambios quedaron y otros se revirtieron.

1. **Logs vacíos y chequeos que no abortaban.** Faltaba el soporte de `printf` para flotantes en el enlazado. Además, los chequeos de campo débil y de error de ajuste estaban comentados (`//return false;`), así que la calibración siempre decía "OK". Se corrigieron las dos cosas.

2. **Primera hipótesis, entrehierro/imán débil: descartada en parte.** Con un `main.cpp` temporal de lazo abierto, girando a mano, no aparecía `weak_field`. El problema no era un campo débil en todos lados.

3. **Corriente de homing muy baja y orden del retroceso.** `HOMING_CURRENT = 0.15`, por debajo del mínimo de 0.4 que recomienda el propio `hw_config.h`. Además, el retroceso posterior al homing (el movimiento más rápido de la secuencia) se hacía antes de restaurar la corriente. Se corrigieron ambos. No era la causa principal.

4. **El motor "ni se movía" y se sentía libre, sin torque.** Tras varias pruebas, un script de MicroPython con paso completo movió el motor perfectamente. Al comparar el cableado del script con `hw_config.h` apareció la causa: **la placa tiene dos pines PWM, `PWMA → GP4` y `PWMB → GP7`, pero el firmware usaba GP4 para los dos canales y nunca configuraba GP7.** La bobina B no se habilitaba, y con una sola fase el motor no puede girar. **Esta fue la primera causa raíz real.**

5. **Saltos en el arranque y la parada.** En Teleplot, `weak_field` y `overspeed` se concentraban al principio y al final de cada vuelta. Se agregó una rampa de aceleración al `main.cpp` temporal.

6. **Bucle infinito en homing.** `G28` avanzaba un poco y después retrocedía a pasos, imprimiendo "End stop not detected" sin parar. Cuando la búsqueda fallaba, el estado nunca se marcaba como terminado, y cada vuelta del bucle repetía el retroceso. Se corrigió.

7. **`rms_error` absurdo (642°).** El error RMS se calculaba como `sqrt(suma)` en vez de `sqrt(suma/N)`, y salía inflado unas 45 veces. Corregido, el error real era de 8–10°.

8. **Hipótesis equivocadas sobre ese error de 8–10°:**
   - *"El barrido frena y arranca en cada muestra."* Se cambió a barrido continuo. No mejoró, pero el cambio quedó porque es más limpio.
   - *"Histéresis por fricción entre ida y vuelta."* Ajustando ida y vuelta por separado, cada una daba ~15°. Descartada.

9. **Causa raíz real de la calibración: saltos de 180° del encoder.** Se capturaron los datos crudos con un script propio (`tools/capture_calibration.py`). El motor giraba bien, pero **la lectura del MT6835 saltaba casi exactamente medio período (180°, `2^20` cuentas) de forma intermitente, con cero errores de CRC.** O sea, el dato llegaba bien por SPI y era el propio chip el que elegía mal la mitad del período. Con el campo débil y deformado de la rueda multipolar, falla la decisión interna del chip sobre en qué mitad está. El firmware tomaba cada salto como movimiento real y acumulaba error.
   - Primera solución: desenrollar la posición módulo medio período. El error bajó de 22.5° a 2.0°.
   - Algunos saltos pasan por valores intermedios. Se agregó el descarte de lecturas con movimiento imposible. El error bajó a 0.49°.
   - Los 0.49° restantes eran el atraso físico del rotor respecto del campo comandado (~0.45°), que cambia de signo entre ida y vuelta. Se cambió el criterio de calidad para descontarlo. Resultado: **0.13–0.15°, calibración OK y repetible.**

10. **`G1` no movía el eje.** Había tres causas:
    - **Lazo de cascada activado por defecto** sin encoder lineal. La "posición lineal" era la misma posición del motor, así que el integrador corría el objetivo hasta +2 mm. Con la cascada desactivada, `G1 X0.2` llegó a 0.205 mm.
    - **La calibración cubría solo 83°** (~0.46 mm), y fuera de ese rango las tablas se congelaban. Se pasó a calibrar una vuelta completa, con la tabla periódica.
    - **`M114` leía el encoder por SPI desde el Core 0** mientras el Core 1 también lo leía. Se corrigió.

11. **La zona débil de la rueda (~212–219°).** Al calibrar la vuelta completa apareció una zona donde el chip da valores corridos 0.1–0.25 de período durante decenas de milisegundos. Se probaron cuatro variantes del filtro del encoder:

    | Variante | Resultado |
    |---|---|
    | Ventana por tiempo, 40 períodos/s | La mejor: ~2 de cada 3 calibraciones OK |
    | Ventana más amplia (100 períodos/s) | Peor |
    | Ventana con techo de 1/8 de período | Peor |
    | Predicción por velocidad | Rota: se desengancha cuando el eje frena de golpe |

    Quedó la mejor variante, con reintento automático de la calibración. **En lazo cerrado, esa zona sigue haciendo perder el seguimiento: el eje se traba en ~1.17 mm.**

---

## 4. Cambios por archivo

### 4.1 `platformio.ini`

```ini
build_flags =
    -Wl,-u,_printf_float
```

**Por qué:** el toolchain enlazaba un `printf` sin soporte de flotantes, así que todos los `%f` de los logs (`LOG_*` usa `vsnprintf`) salían vacíos. Sin esto era imposible ver el `rms_error` y otros valores.

### 4.2 `src/hw_config.h`

| Constante | Antes | Ahora | Por qué |
|---|---|---|---|
| `PIN_MOTOR_PWMAB` | `4` (usado para los dos canales) | Reemplazado por `PIN_MOTOR_PWM_A = 4` y `PIN_MOTOR_PWM_B = 7` | La placa tiene PWMA en GP4 y PWMB en GP7. Sin GP7, la bobina B nunca se habilitaba y el motor no giraba. **Fix principal del motor.** |
| `HOMING_CURRENT` | `0.15` | `0.4` | Estaba por debajo del mínimo recomendado (0.4) y no alcanzaba para mover la mecánica acoplada. |
| `CALIBRATION_RANGE` | `83` grados | `370` grados | 83° cubrían solo ~0.46 mm. La tabla ahora se hace periódica y necesita al menos una vuelta completa. |
| `ENCODER_LUT_SIZE` | `256` | `1024` | Con una vuelta completa hace falta más resolución (~0.36° por celda) para seguir la ondulación de la rueda, cuyo período es ~6° de eje. |

### 4.3 `src/main.cpp` (firmware real) y `main_firmware_completo.cpp.bak`

En la construcción de `TB6612MotorDriver`, el cuarto argumento de cada canal pasó a ser `PIN_MOTOR_PWM_A` y `PIN_MOTOR_PWM_B`, en vez de `PIN_MOTOR_PWMAB` para los dos. `main_firmware_completo.cpp.bak` (en la raíz, fuera de `src/` para que PlatformIO no lo compile) es la copia de respaldo del firmware real que se guardó mientras `src/main.cpp` tenía el firmware temporal. Hoy `src/main.cpp` vuelve a ser el firmware real.

### 4.4 `src/hardware/MT6835_encoder.cpp` / `.h`: desenrollado robusto del ángulo

**Problema:** frente a la rueda multipolar, el chip a veces reporta el ángulo corrido 180° (medio período), a veces pasando por valores intermedios, sin que el eje se mueva. El desenrollado original (módulo un período completo) tomaba cada salto como movimiento real.

**Cambio en `update_abs_raw_angle()`:**

1. La diferencia entre lecturas se reduce **módulo medio período** (`CPR/2`), en el rango `[-CPR/4, CPR/4)`. Un salto de 180° ya asentado se absorbe solo. Como la rueda tiene muchos períodos por vuelta, nunca hace falta saber en qué mitad del período está: solo cuánto se movió.
2. **Se descarta una lectura** si implica un movimiento más rápido que el máximo físico posible desde la última lectura aceptada. Así se filtran los valores intermedios de un salto. El criterio es por tiempo, para que valga igual con lecturas cada pocos µs (lazo servo) que cada ~0.5 ms (calibración):
   ```
   max_step = NOISE_COUNTS + dt_us × MAX_PERIODS_PER_S × CPR / 1e6
   ```
3. Si pasan `RESYNC_US` sin lecturas aceptables, se acepta la siguiente igual para resincronizar.

**Constantes actuales:**

| Constante | Valor | Significado |
|---|---|---|
| `NOISE_COUNTS` | `CPR >> 10` = 2048 cuentas | Margen de ruido (~0.01° de eje) |
| `MAX_PERIODS_PER_S` | 40 | ~1.3 vueltas/s con la rueda de 30 períodos. El lazo servo limita a 1 vuelta/s. |
| `RESYNC_US` | 20000 (20 ms) | Tiempo sin lecturas válidas antes de resincronizar |

**Header:** nuevo miembro `uint64_t last_accepted_time_us`.

**Restricción:** entre dos lecturas aceptadas, el eje tiene que moverse menos de un cuarto de período (~3° de eje). Siempre se cumple con lecturas frecuentes, pero no si el motor se mueve sin que nadie lea el encoder.

### 4.5 `src/servo_control/actuator_calibration.cpp`: medición y armado de tablas

Es el archivo con más cambios. En orden:

1. **Chequeos que abortan de verdad.** Antes solo avisaban y la calibración siempre "pasaba":
   - Campo débil en más del 5 % de las muestras → aborta. El mensaje ahora usa el total real de muestras (ida + vuelta).
   - Error de ajuste de la tabla mayor a 0.5° → aborta.
2. **Barrido continuo.** En vez de llamar a `rotate_field()` una vez por muestra (el motor frenaba y arrancaba ~2000 veces), se hace un único movimiento por pasada (ida y vuelta) y se muestrea desde el callback `on_step`.
3. **Lectura del encoder en cada paso interno** (cada ~0.5 ms), pero se guarda solo 1 de cada `sample_decimation = 8`. El desenrollado del encoder necesita lecturas frecuentes.
4. **Salida Teleplot con `M56 P`:** además de la línea CSV, imprime `>motor_pos_deg`, `>encoder_angle_raw` y `>weak_field`.
5. **Criterio de calidad sin el atraso del rotor.** La nueva función `rms_error_without_direction_offset()` calcula el RMS del error de la tabla restando el error medio de cada sentido. El atraso del rotor (`motor_pos` sale del campo comandado, y el rotor va ~0.45° atrás en cada sentido) no es un error de la tabla: la tabla combinada lo promedia, que es lo correcto. El log informa ambos valores:
   ```
   I)LUT 'encoder_raw_angle -> motor_pos': rms_error = 0.136 deg, atraso del rotor = -0.452 deg
   ```
6. **Tabla periódica para todo el recorrido:**
   - Se calcula por mínimos cuadrados la pendiente (cuentas de encoder por rad de motor) y de ahí los períodos por vuelta.
   - La rueda es un anillo cerrado, así que el número tiene que ser **entero**. Si difiere más de 0.2 del entero más cercano, el encoder perdió el seguimiento y la calibración aborta.
   - Se verifica que el rango calibrado cubra al menos una vuelta.
   - `set_period(N × CPR, ±2π)`: fuera del rango calibrado, la tabla se repite cada vuelta.
   - Log: `I)Rueda magnetica: 29.94 periodos de encoder por vuelta de motor`.
7. **La tabla `motor_pos → field_angle` pasa a ser lineal**, de 2 puntos sobre ±200 rad de motor (más que los 78.5 rad de todo el recorrido). Por construcción es exactamente `campo = campo_inicial + motor_pos × pares_de_polos`, así que ajustarla con datos era innecesario, y además se congelaba fuera del rango calibrado. Se eliminó el vector `motor_pos_and_field_angle`.
8. **Memoria:** `reserve()` del vector de muestras antes de empezar (~7000 muestras con una vuelta completa), para evitar una realocación a mitad del barrido que duplicaría la memoria por un instante.
9. **Regreso al inicio más lento:** `3 × field_velocity` en vez de `40·2π` rad/s, para que un patinazo no haga perder el seguimiento al encoder.
10. Se agregó `#include <Arduino.h>` para `Serial`.

### 4.6 `src/utilities/lookup_table.cpp` / `.h`

1. **Bug del RMS:** `rms_error = sqrt(total_loss)` pasó a `sqrt(total_loss / N)`. Antes el error salía inflado ~45× (√2048), por eso aparecían valores como "642°".
2. **`optimize_lut()` reescrita como mínimos cuadrados exactos.** Cada muestra afecta solo dos celdas vecinas de la tabla, así que las ecuaciones normales son tridiagonales y se resuelven en O(N) con el algoritmo de Thomas, en `double`. Un término de regularización chico (`λ = 1e-4`) tira hacia el valor inicial y deja definidas las celdas sin muestras. Reemplaza a las 1000 iteraciones de descenso por gradiente, que con ~7000 muestras habrían tardado decenas de segundos en el RP2040. Se verificó en la PC que el resultado coincide con `numpy.linalg.lstsq`. La firma pasó a recibir `const&` (antes copiaba el vector).
3. **Tabla periódica:** nuevos miembros `period_in` y `period_out`, y el método `set_period()`. En `evaluate()`, si `x` cae fuera del rango, se desplaza un número entero de períodos y se suma el mismo número de `period_out` a la salida.
4. **Guardado y carga del período:** `save_lut_to_file()` escribe `period_in` y `period_out` al final del archivo. `load_lut_from_file()` los lee si están, así que los archivos viejos siguen cargando.

### 4.7 `src/servo_control/homing_controller.cpp`

1. **Bucle infinito cuando no encuentra el tope.** Antes, la rama "fuera de rango de búsqueda" ponía `search_failed = true` y llamaba a `finalize()` sin marcar `state = Done`. `run_blocking()` seguía llamando a `update()` para siempre, y cada llamada repetía el retroceso. Ahora llama a `on_endstop_detected()`, que marca `Done` y desactiva el motor cuando `search_failed`.
2. **`finalize()` no retrocede si la búsqueda falló:** si nunca se encontró un tope, no hay de dónde retroceder.
3. **Corriente antes del retroceso:** `set_amplitude_smooth(initial_current)` ahora va **antes** del `rotate_field()` de retroceso. Antes, ese movimiento (el más rápido de la secuencia, 100 rad/s de campo) se hacía con la corriente reducida del homing.

### 4.8 `src/axis_controller.cpp` / `.h`

1. **Reintento de la calibración:** `calibrate()` intenta `measure_calibration_data()` hasta 3 veces. Si un intento falla, avisa con `W)AxisController: reintentando calibracion (2 de 3)`.
2. **Se eliminó `read_position_mm()`:** leía el encoder por SPI desde el Core 0 mientras el Core 1 lo leía en paralelo.

### 4.9 `src/command_processor.cpp`

1. **`M17` exige calibración válida.** Antes se podía activar el lazo cerrado sin calibración, con tablas por defecto o parciales, y el lazo se disparaba. Ahora responde `error: eje no calibrado, correr M56 primero`.
2. **`M114`** usa `get_position_mm()`, la posición que ya calcula el Core 1, en vez de leer el encoder.
3. **`M119`** agrega una segunda línea con el estado interno del servo:
   ```
   servo: motor_update=1 motor_pos=1.6799 rad vel=0.000 rad/s out=-1.414 field=83.525 amp=0.60
   ```
   - `motor_update`: 1 si el Core 1 está comandando el motor (después de `M17`).
   - `motor_pos`: posición del motor en rad (0 = inicio de la calibración).
   - `vel`: velocidad estimada.
   - `out`: salida del lazo de velocidad, en rad de ángulo eléctrico de adelanto. ±1.414 (±81°) es la saturación.
   - `field`: ángulo de campo que se está aplicando.
   - `amp`: amplitud de corriente (0–1).

### 4.10 `src/servo_control/servo_controller.h`

Nuevos getters para `M119`: `get_velocity()`, `get_output()`, `is_motor_update_enabled()`.

### 4.11 `src/shared_data.h`

`cascade_corrector_enabled` cambió de `true` a **`false`**. No hay encoder lineal: `read_linear_encoder()` devuelve la misma posición del motor que ya corrige el lazo servo. Con eso el integrador de la cascada solo acumulaba el error de seguimiento y corría el objetivo hasta +2 mm (`kp=0.5`, `ki=2`, límite de ±1 mm). Hay que volver a activarlo cuando exista un encoder lineal real.

---

### 4.12 Ajustes al montaje real (husillo de bolas, paso 2 mm, 260 mm de recorrido)

Con el recorrido real (~260 mm = 130 vueltas de motor) aparecieron límites que con 25 mm no se notaban:

| Archivo | Cambio | Por qué |
|---|---|---|
| `hw_config.h` | `AXIS_MAX_POSITION_MM`: 25 → **255** | Recorrido útil medido ~260 mm, con 5 mm de margen al tope opuesto. |
| `hw_config.h` | `HOMING_VELOCITY`: 1 → **3 rad/s** (~0.95 mm/s) | A 1 rad/s, recorrer 260 mm tardaría ~14 min. Con 3 rad/s tarda ~4.5 min en el peor caso. No conviene pasar de ~6 rad/s (1 vuelta/s): el filtro del encoder descarta movimientos más rápidos (`MAX_PERIODS_PER_S = 40`). |
| `hw_config.h` | Nueva constante `HOMING_SEARCH_RANGE_MM = AXIS_MAX_POSITION_MM + 15` | Antes el homing buscaba el tope en **una sola vuelta de motor** (360°, ~2 mm). Con el carro lejos del tope, o fallaba o, peor, detectaba un "tope" falso. |
| `axis_controller.cpp` | `home()` usa `mm_to_motor_pos_rad(HOMING_SEARCH_RANGE_MM)` como rango de búsqueda | Ídem. |
| `MT6835_encoder.h/.cpp` | `AbsRawAngleType`: `int32_t` → **`int64_t`**. También `reset_abs_angle()` y `read_abs_angle()` | 130 vueltas × 30 períodos × 2^21 cuentas ≈ 8.2e9, que desborda un `int32` (2.1e9) pasados ~68 mm. |
| `servo_controller.h/.cpp` | La lectura del encoder en `update()` y `encoder_angle_to_motor_pos()` usan `AbsRawAngleType` | Antes truncaban a `int32`. |
| `homing_controller.h` | `field_angle_offset` y `last_eval_field_angle_offset`: `float` → **`double`** | Con todo el recorrido llegan a ~41.000 rad, donde un `float` ya no puede sumar el incremento de cada iteración (~0.005 rad). El campo avanzaría a saltos o se quedaría quieto. |
| `homing_controller.cpp` | El tope se acepta solo con **15 ventanas de evaluación seguidas** con el encoder casi quieto (`STALL_WINDOWS_REQUIRED`, ~11° de eje), no con una sola | La zona débil de la rueda (~7° de eje) se cruza en cada vuelta, 130 veces en el recorrido. Ahí el encoder puede descartar lecturas y parecer detenido: con una sola ventana se detectaba un tope falso. Probablemente eso explica que el ángulo de home cambiara tanto entre sesiones. |
| `homing_controller.cpp` | `finalize()` retrocede además lo que avanzó el campo mientras se confirmaba el bloqueo (`STALL_WINDOWS_REQUIRED × eval_field_angle_delta`) | Si no, el campo quedaba ~9° de eje "adentro" del tope y los primeros grados del barrido de calibración tenían al rotor todavía apretado contra el tope. |
| `actuator_calibration.cpp` | Tabla lineal posición → campo: ±200 → **±1000 rad** de motor | Tiene que cubrir los ~817 rad del recorrido completo. |

**Cómo detecta los topes:** no hay finales de carrera. El homing gira despacio y con corriente reducida hacia `D` (por defecto `D-1`) y declara el tope cuando el encoder deja de avanzar respecto de lo comandado. Ese punto es el 0 mm. El tope opuesto no se detecta: el límite de ese lado es solo por software (`AXIS_MAX_POSITION_MM`).

**Pendiente de verificar en el montaje:**
- Que el homing encuentre el tope real desde la otra punta del riel.
- Que ningún cruce de la zona débil dispare un tope falso.
- Que el lazo cerrado siga posicionando bien a distancias grandes. Con `float`, cerca de 255 mm la posición del motor tiene una resolución de ~6e-5 rad, y la estimación de velocidad se vuelve más ruidosa.

### 4.13 Rueda de imanes floja: protecciones

**Hallazgo en el montaje:** la rueda de imanes **no está fija al eje del motor** (no está pegada al hub). Con el homing llegando al tope, el motor vibraba contra el tope, la rueda suelta seguía girando y el encoder "veía" movimiento: el homing no detectaba el bloqueo y seguía empujando.

Con la rueda floja, el encoder mide la rueda y no el rotor. Esto también es un candidato fuerte para el atasco del lazo cerrado en ~1.17 mm (sección 7.1) y para las calibraciones que variaban entre corridas: si la rueda patina, la tabla encoder → rotor deja de valer y la conmutación se desalinea. Todas las calibraciones midieron 29.93–29.95 períodos por vuelta en vez de exactamente 30. Es compatible con que la rueda se atrase ~0.2 % respecto del eje, aunque también podría deberse a cómo cambia el atraso del rotor a lo largo del barrido.

**Esto no se puede corregir en firmware: hay que fijar la rueda al eje** (prisionero sobre un plano del eje, pegamento, chaveta). Lo que sí se agregó es que el firmware lo detecte y se detenga:

| Archivo | Cambio |
|---|---|
| `homing_controller.cpp/.h` | La relación encoder/campo se evalúa sobre **un período completo del encoder** (17 ventanas ≈ 12° de eje, `EVAL_WINDOWS_PER_CHECK`), no sobre ventanas de 0.72°. En ventanas cortas, la no-linealidad de la rueda hace variar la relación entre 0 y 2 aunque todo esté bien. Reglas: relación < 0.3 → **tope encontrado**; 0.7–1.3 → movimiento normal; cualquier otro valor 3 veces seguidas → **homing abortado** con `E)Homing abortado: encoder no sigue al motor (...). Rueda floja?` y motor apagado. Reemplaza al criterio anterior de 15 ventanas cortas. |
| `homing_controller.cpp` | El retroceso final compensa hasta dos chequeos completos de avance del campo contra el tope. |
| `axis_controller.cpp/.h` | **Corte por error de seguimiento en lazo cerrado:** si el eje queda a más de 1 mm del objetivo durante más de 0.5 s con el lazo activo, el Core 1 deshabilita el lazo y pone la corriente en 0 (`servo_fault`). `M17` borra la falla. |
| `command_processor.cpp` | `M119` muestra `fault=0/1` en la línea `servo:`. |

### 4.14 Costura de la tabla periódica y velocidad desde las cuentas

**Problema:** con el motor quieto en el home, `M119` mostraba velocidades de ±68–96 rad/s. La tabla mide ~29.95 períodos por vuelta (cada pasada arranca desde parado y el atraso del rotor tarda en establecerse), pero se repite cada 30 exactos. En la costura, la posición saltaba ~0.5°. El home queda justo en la costura, porque la calibración arranca ahí, así que el ruido del encoder hacía saltar la posición de un lado al otro, y cada salto en ~20 µs aparecía como una velocidad enorme.

| Archivo | Cambio |
|---|---|
| `actuator_calibration.cpp` | Después del ajuste se mide el desfase en la costura, `LUT(min + período) − LUT(min) − 2π` (típico: −0.44°), y se reparte linealmente sobre la vuelta, para que la tabla empalme exacta consigo misma. El log lo informa: `Desfase en la costura de la vuelta: ... deg (se corrige)`. |
| `lookup_table.cpp` | Con período configurado, `evaluate()` siempre reduce `x` a la primera vuelta `[min, min + período)` (antes solo fuera de `[min, max]`), así la única costura es la que quedó corregida. `init()` e `init_interpolating()` resetean el período. Nuevos getters `get_period_in()` / `get_period_out()`. |
| `servo_controller.cpp/.h` | Con tabla periódica, la velocidad se calcula de la diferencia de **cuentas enteras** del encoder × (2π / cuentas por vuelta), no de la salida de la tabla. No le afecta ninguna costura ni la pérdida de resolución del `float` lejos del home. Nuevo miembro `encoder_angle_raw_prev`. |

Resultado: velocidad en reposo = 0.000 rad/s.

**Prueba de lazo cerrado posterior (con la rueda todavía floja):** después de `M17`, `G1 X0.5` no movió el eje. La salida quedó saturada (`out=+1.414`), el campo fijo y el encoder casi quieto, y a los 0.5 s saltó la protección (`fault=1`). Es la firma de una rueda que no acompaña al rotor ante un tirón de torque: el rotor avanza ~1.6°, la rueda no, el lazo no actualiza el campo y todo queda congelado. En la calibración el movimiento es suave y la rueda se arrastra por fricción, por eso la calibración pasa igual.

### 4.15 Umbrales ajustados al montaje real

Con el montaje sin cambios, la calibración empezó a fallar con `Magnetic field too weak for 402 of 7757` (5.2 %), repetido casi igual en los 3 intentos (402, 404, 403). Antes daba 4.1 %. No es ruido ni un cambio mecánico: el campo débil de esta rueda está naturalmente entre 4 y 5.5 %, según dónde caiga el home (los 10° que sobran del barrido de 370° a veces caen sobre la zona débil y la cuentan dos veces). En paralelo, un `G28` abortó con relación encoder/campo 0.325 en un tope real: el rotor vibra y salta de polo contra el tope, y el encoder registra algo de movimiento.

| Archivo | Cambio |
|---|---|
| `actuator_calibration.cpp` | Campo débil: **aborta solo por encima del 20 %**. Entre 5 y 20 % solo avisa (`W)Campo debil en X de Y muestras (N%)...`). La validez de los datos la controlan el error de la tabla (< 0.5°) y los períodos enteros por vuelta. |
| `homing_controller.cpp` | `STALL_RATIO`: 0.3 → **0.5**. En movimiento libre la relación es ~1 (sano entre 0.7 y 1.3). |

### 4.16 Encoder montado según el diseño original: se revierten los parches de orientación

**Hallazgo:** el encoder estaba montado girado 90° respecto del diseño original de MicroManipulatorStepper. En el diseño, la placa está parada, tangente al borde de la rueda y paralela al eje de los imanes, así que el die mide en el plano tangencial + axial (el **XZ** de la simulación). En el montaje anterior, la placa estaba paralela a la cara de la rueda, y el die medía en el plano tangencial + radial (**XY**).

Con el modelo magpylib del notebook `SIMULACIONES_CLEAN.ipynb` (imanes de 3 × 5 mm N35, paso 3 mm):
- en el punto óptimo, con el die en XZ: 0° de error y 58 mT;
- con el die en XY: 14° de error y 37–58 mT, y más centrado sobre la fila, errores de hasta 98° y campo que cae a ~0.

Eso explica los saltos de 180°, el `weak_field` y el overspeed que se midieron. El montaje ya se corrigió al patrón original.

**Revertido**, porque eran parches para las lecturas malas del montaje girado:

| Qué | Vuelve a |
|---|---|
| Desenrollado del encoder módulo medio período + descarte de lecturas por velocidad (`max_periods_per_s`, `set_max_speed`) + contadores `enc_rechazos` / `enc_resync` | Desenrollado original por período completo. **Se mantiene `AbsRawAngleType` en `int64_t`**, necesario para los 260 mm. |
| Límites de velocidad del filtro por modo (homing, calibración, servo) en `axis_controller.cpp` | Eliminados. |
| En `ServoController::update`: sostener el campo sin adelanto y congelar los PID cuando el encoder deja de dar lecturas confiables | Eliminado. |
| Campo débil: aborto recién por encima del 20 % (aviso desde el 5 %) | **Aborta si más del 5 %** de las muestras tienen `weak_field`. |
| Reintento automático de la calibración (3 intentos) | Un solo intento. |
| Pruebas de `BW` 6 y 7 del MT6835 | `BW = 5`, el original (`encoder->init(0x5, 0x4)`). |

**Se mantiene**, porque no depende de la orientación: pines PWM, parámetros del husillo y del recorrido, homing de recorrido completo con detección de rueda floja (`STALL_RATIO = 0.5`), tabla periódica con la costura corregida y ajuste exacto, velocidad desde las cuentas del encoder, `dt` mínimo en el Core 1, descuento del atraso del rotor, cascada desactivada, corte por falla, `M17` con calibración obligatoria, estado del servo en `M119` y la traza `M121`.

### 4.17 Traza del lazo cerrado (`M121`)

`AxisController` guarda en el Core 1 una muestra por ms mientras el lazo está activo, en un buffer circular de 2000 muestras (~2 s). Cada muestra tiene tiempo, objetivo, posición, velocidad, salida, lectura cruda del chip, status y cantidad de iteraciones del lazo. La traza se congela 300 ms después de que el error pase por primera vez de 0.1 mm, y también cuando salta el corte por falla. Así queda registrado el momento en que el eje se traba. `M121` la imprime como CSV, y `M17` la reinicia. De ahí salió, por ejemplo, que el lazo corre a ~22 kHz (~45 µs por iteración).

## 5. Herramientas nuevas (`tools/`)

Las dos necesitan el puerto serie libre: cerrar antes Teleplot, `pio device monitor` o cualquier otra terminal. Linux permite un solo proceso por puerto.

### 5.1 `tools/capture_calibration.py`

Resuelve el problema de "la terminal se borra y no puedo guardar los puntos".

```bash
python3 tools/capture_calibration.py --out calib
```

- Manda `M56 P` y captura toda la salida.
- Genera:
  - `calib.csv`, con columnas `motor_pos_rad, field_angle_rad, encoder_raw, weak_field`
  - `calib_log.txt`, con el log completo
  - `calib.png`, con 4 gráficos: encoder contra posición, residuo contra una recta, residuo según la fase dentro del período del encoder, y salto de encoder por muestra con marcas de `weak_field`
- Imprime por pasada (ida y vuelta): pendiente, períodos por vuelta, residuo RMS y máximo, y cantidad de pasos donde el encoder retrocede.
- Con `--from-log archivo.txt` analiza un log ya capturado, sin mandar nada.

**Cómo leer el gráfico:** con todo sano, el panel 1 es una recta con ida y vuelta superpuestas, y el panel 4 no tiene picos grandes. Saltos de ~±1 000 000 cuentas en el panel 4 son saltos de 180° del encoder.

### 5.2 `tools/send_gcode.py`

Manda comandos uno por uno y muestra las respuestas:

```bash
python3 tools/send_gcode.py "M56" "M17" "G1 X0.5" "wait 2" "M119" "M18"
```

`wait N` espera N segundos mostrando lo que llega. Las líneas Teleplot (`>...`) se ocultan.

---

## 6. Firmware temporal de prueba (lazo abierto)

Durante la depuración, `src/main.cpp` se reemplazó varias veces por un firmware mínimo de lazo abierto, sin cores, sin G-code y sin lazo cerrado, para probar motor y encoder aislados. Su última versión:

- Inicializa encoder y motor (amplitud 0.8) y queda esperando.
- Arranca con el **botón de usuario** (`PIN_USER_BUTTON`, GPIO 24, activo en bajo) o con el comando **`G56`** por serie. La caja "Send" de Teleplot sirve para mandarlo sin abrir otra terminal.
- Hace `NUM_REVOLUTIONS = 4` vueltas mecánicas, con pasos de `0.1257 rad` de campo cada `2 ms` (~20 s en total), y rampa de aceleración y frenado en los primeros y últimos 200 pasos.
- Imprime en formato Teleplot: `>raw_angle`, `>encoder_angle_deg`, `>field_angle_rad`, `>weak_field`, `>overspeed`, `>undervolt`.

Hoy `src/main.cpp` tiene el firmware real. El temporal no está guardado como archivo aparte; si se vuelve a necesitar, hay que rearmarlo con esta descripción.

**Ojo al leer esos gráficos:** `field_angle_rad` es el ángulo *comandado* y crece lineal aunque el motor no se mueva. El movimiento real lo muestra solo `raw_angle`.

---

## 7. Problemas abiertos y próximos pasos

### 7.1 El lazo cerrado se traba en ~1.17 mm (prioridad alta)

- **Síntoma:** `G1 X2.5` llega a 1.176 mm y se queda. `M119` muestra `out` saturado (±1.414) y `vel = 0`. Tampoco retrocede.
- **Causa:** es la zona de ~212° de la vuelta del motor, donde el campo de la rueda es muy débil. Ahí el encoder pierde el seguimiento, la posición queda corrida y la conmutación deja de estar alineada con el rotor, así que el motor queda sin torque útil.
- **Arreglo recomendado (físico):**
  1. Con el motor apagado (`M18`), llevar el eje a mano a ~1.17 mm desde el tope.
  2. Revisar el entrehierro en ese punto y si hay un imán flojo, girado o más débil, o si la rueda está descentrada ahí.
  3. Reducir el entrehierro en general debería eliminar los saltos de 180° desde el origen.
  4. Verificar con `tools/capture_calibration.py`: en `calib.png` no deberían aparecer picos cerca de los 215° ni `weak_field` en esa zona.
- **Alternativa en software (no resuelta):** un observador de seguimiento (PLL o Kalman) con límite de aceleración. El intento con predicción por velocidad se desenganchaba en las frenadas bruscas. La dificultad es que las excursiones del chip (~0.1 período durante varios ms) se parecen en magnitud y duración a una frenada real.

### 7.2 Calibración guardada en flash

`load_calibration()` marca el eje como calibrado sin volver a validar. La tabla depende de que el homing termine en el mismo punto del período del encoder, y con el desenrollado módulo medio período puede quedar corrida medio período entre sesiones. Antes de confiar en `M56 S` hay que verificar que después de un reinicio, `G28` más la tabla guardada den la misma posición que una calibración nueva. **Mientras tanto: correr `M56` en cada arranque.**

### 7.3 Homing en el sentido opuesto (`D1`)

`G28 D1` y `M56 D1` no funcionan bien hoy: la calibración siempre barre en sentido positivo desde el home y los mm positivos siempre apuntan en ese sentido. Con `D1` el barrido empujaría contra el tope recién encontrado. Para soportarlo hay que invertir el sentido del barrido y de las coordenadas según `D`.

### 7.4 Sintonía del PID

En reposo, la salida del lazo de velocidad suele quedar saturada (`out = ±1.414`) aun con errores de pocos µm. `POS_KI = 30000` lleva el integrador de posición a su límite casi enseguida. El posicionamiento funciona (error de ~5–35 µm en las pruebas), pero conviene resintonizar con `M55` una vez resuelto el punto 7.1.

### 7.5 Otros detalles

- `MOTOR_MAX_CURRENT_FACTOR = 0.6` y la fuente de 10 V / 1 A anduvieron bien en todas las pruebas una vez corregido el pin PWM.
- `ENCODER_MAGNET_PITCH` y `ENCODER_MAGNET_RADIUS` dan 30.3 períodos por vuelta, y la medición da 29.94 ≈ 30. Son coherentes y solo se usan en el homing.

---

## 8. Secuencia de uso actual

```
M56          homing (hasta ~4.5 min si el carro arranca en la otra punta) + calibración (~45 s). Esperar "calibracion OK".
M17          activa el lazo cerrado (exige calibración válida)
M119         ver estado; motor_update=1
G1 X0.5      mover a 0.5 mm (absoluto, 0 = tope, rango 0–255 mm)
G1 X0.2      volver hacia el tope
M114         posición actual
M18          apagar el motor
```

Movimientos relativos: `G91`, después `G1 X-0.3`, y `G90` para volver a modo absoluto.

**Hasta resolver 7.1, quedarse entre 0 y ~1.1 mm.**

---

## 9. Datos de referencia medidos

| Magnitud | Valor |
|---|---|
| Períodos de encoder por vuelta de motor | 29.94 (entero: 30), coherente con radio 28.9 mm y paso 3 mm |
| Error de la tabla encoder → posición (sin atraso) | 0.13–0.15° RMS |
| Atraso del rotor respecto del campo (calibración a 20 rad/s de campo) | ~0.45–0.50° |
| Ondulación del encoder antes de corregir con la tabla | ±1–1.5°, período ~6° de eje (medio período del encoder) |
| Ajuste por sentido (ida o vuelta sola) | 0.08–0.10° RMS |
| Muestras con `weak_field` en una vuelta completa | ~4 % (el límite de aborto es 5 %) |
| Zona de campo más débil | ~212–219° de la vuelta del motor (~1.17 mm desde el home) |
| Duración de `M56` con vuelta completa | ~45 s |
