# 5. Apéndice

Material para el apéndice del informe: equivalencias de unidades, parámetros de la versión final, comandos, formato de telemetría y cronología del trabajo.

---

## 5.1 Equivalencias (husillo de 5 mm)

| Magnitud | Equivale a |
|---|---|
| 1 vuelta de motor | 360° de motor = **5 mm** = 30 períodos del encoder = 60 imanes = 50 ciclos eléctricos = 200 pasos |
| 1 período del encoder (2 imanes, 360° del sensor) | 12° de motor = **166,7 µm** = 2²¹ cuentas |
| 1 imán | 6° de motor = **83,3 µm** |
| 1 ciclo eléctrico | 7,2° de motor = **100 µm** |
| 1 paso completo | 1,8° de motor = **25 µm** |
| 1° de motor | 13,89 µm |
| 1° del sensor (encoder) | 0,0333° de motor = **0,463 µm** |
| 1° eléctrico | 0,02° de motor = **0,278 µm** |
| 1 rad de motor | 0,796 mm |
| 1 cuenta del encoder | **0,0795 nm** |
| Unidad de posición entre núcleos y en telemetría | 0,25 nm (`int32`, ±536 mm) |

Frecuencias a 2 mm/s: vuelta 0,4 Hz · período del encoder 12 Hz · imán 24 Hz · ciclo eléctrico 20 Hz · paso 80 Hz.

Fórmulas:
```
µm de carro = grados del sensor × 0,463 = grados de motor × 13,89 = grados eléctricos × 0,278
rad eléctricos = 50 × rad de motor
períodos por vuelta = |d(cuentas)/d(rad de motor)| × 2π / 2²¹   (→ 30)
```

---

## 5.2 Parámetros de la versión final (`src/hw_config.h`, 06/11/2026)

| Grupo | Parámetro | Valor |
|---|---|---|
| Mecánica | `LEAD_SCREW_PITCH_MM` | 5 mm/vuelta |
| | `AXIS_MIN/MAX_POSITION_MM` | 0 / 255 mm |
| Perfil | `DEFAULT_MAX_VELOCITY_MM_S` / `DEFAULT_MAX_ACCEL_MM_S2` | 2 mm/s / 5 mm/s² |
| | `MAX_VELOCITY_LIMIT_MM_S` (tope de `M203`) | 5 mm/s |
| Motor | `MOTOR_POLE_PAIRS` | 50 |
| | `MOTOR_MAX_CURRENT_FACTOR` | 0,6 |
| Encoder | `ENCODER_MAGNET_COUNT` / períodos por vuelta | 60 / 30 |
| | Configuración del chip | BW 5, histéresis 4, SPI 8 MHz, CRC desactivado |
| Homing | `HOMING_VELOCITY` / corriente | 2 rad/s de motor (1,6 mm/s) / 0,6 |
| Calibración | `CALIBRATION_RANGE` / `CALIBRATION_TRANSIENT_SKIP` | 400° / 15° |
| | `CALIBRATION_FIELD_VELOCITY` | 20 rad/s eléctricos |
| | `ENCODER_LUT_SIZE` | 1024 |
| | `CALIBRATION_FILTER_MOTOR_HARMONICS` | sí |
| Conmutación | Ciclos de verificación / desfase máximo aceptado | 2 / 45° eléctricos |
| Lazo | `POS_KP` / `POS_KI` | 60 / 100 |
| | `VEL_KP` / `VEL_KI` / `VEL_LOWPASS_TC` | 0,12 / 54 / 4 ms |
| | Límite de la salida | ±81° eléctricos (0,45π) |
| | `VELOCITY_FROM_LUT` (B1) / `VELOCITY_FEEDFORWARD` (A1) | sí / sí |
| Anti-cogging | Activo al arrancar / reaprendizaje en `M56` | sí / sí (2 pasadas 0 → 50 → 0 mm, ganancia 0,8) |
| | Velocidad de referencia / fin del desvanecimiento | 2 mm/s / 3 mm/s |
| Seguridad | Corte por falla | \|error\| > 1 mm durante 0,5 s |

Historia de las ganancias:

| Desde | `POS_KI` | `VEL_KP` | `VEL_KI` | Lazo de velocidad |
|---|---|---|---|---|
| inicio (heredado) | 30 000 | 0,2 | 90 | V0 F0 |
| 13/10 | 3 000 | 0,2 | 90 | V0 F0 |
| 16/10 | 1 500 | 0,2 | 90 | V1 F1 |
| 20/10 | 100 | 0,2 | 90 | V1 F1 (+ anti-cogging desde el 20/10) |
| **06/11** | **100** | **0,12** | **54** | **V1 F1 + anti-cogging** |

`POS_KP` = 60 y factor de corriente 0,6 en todas.

---

## 5.3 Comandos

| Comando | Función |
|---|---|
| `M56 [S] [P] [H] [F0/1] [A0/1]` | Homing + calibración + verificación de conmutación + reaprendizaje del anti-cogging. `S` guarda en flash |
| `G28` | Homing contra el tope y verificación de la conmutación |
| `M17` / `M18` | Habilitar el lazo cerrado / apagar el motor |
| `G0` / `G1 X<mm> [F<mm/s>]` | Movimiento con perfil trapezoidal |
| `G90` / `G91` | Absoluto / relativo |
| `M114` / `M119` | Posición / estado completo |
| `M55 A B C D F` | Ganancias en caliente: `POS_KP`, `POS_KI`, `VEL_KP`, `VEL_KI`, constante del filtro |
| `M57 X<mm>` | Medición de la conmutación a lo largo del recorrido |
| `M58 V<0/1> F<0/1>` | Velocidad desde la tabla (B1) / realimentación anticipada (A1) |
| `M59 …` | Anti-cogging: activar, ganancia, tablas, guardar/cargar |
| `M120 S<0/1> P<n>` | Telemetría binaria |
| `M121` | Traza del lazo previa al último corte |
| `M203 X<mm/s>` | Velocidad máxima |
| `G56` | Barrido de caracterización (solo en el firmware de barrido) |

Secuencia típica: `M56` (3–5 min; deja el lazo activo en 0) → `G1 X…`.

---

## 5.4 Telemetría

Encabezado `0xAA 0x56` + 28 bytes, little endian, formato `struct` de Python `<IiiffiBBH`:

| Campo | Tipo | Unidad |
|---|---|---|
| `timestamp_us` | uint32 | µs |
| `target_q` | int32 | objetivo, 0,25 nm |
| `motor_pos_q` | int32 | posición medida, 0,25 nm |
| `vel` | float | rad/s de motor |
| `out` | float | salida del lazo de velocidad, rad eléctricos |
| `chip_raw` | int32 | ángulo crudo del MT6835 (0 … 2²¹ − 1) |
| `status` | uint8 | bits del encoder (overspeed, campo débil, subtensión, CRC) |
| `flags` | uint8 | lazo activo, corte por falla |
| `seq` | uint16 | contador (huecos = paquetes perdidos) |

Antes del 23/10 el encabezado era `0xAA 0x55` con objetivo y posición en `float` [mm]. Las herramientas viejas no leen los datos nuevos ni al revés.

---

## 5.5 Herramientas de PC (Python)

| Script | Para qué |
|---|---|
| `tools/interfaz.py` | Interfaz gráfica (PyQt5 + pyqtgraph): control, telemetría en vivo, grabación CSV |
| `tools/send_gcode.py` | Mandar comandos en secuencia |
| `tools/closed_loop_test.py` | Idas y vueltas con telemetría, resumen y gráficos |
| `tools/analisis_errores.py` | Presupuesto del error por plegado; figuras del reporte del 16/10 |
| `tools/pid_sweep.py`, `ki_sweep.py`, `ki_eleccion.py` | Barridos de ganancias |
| `tools/cogging_learn.py` | Aprendizaje del anti-cogging |
| `tools/encoder_sweep.py`, `plot_periods.py` | Caracterización del encoder |
| `tools/escalera_nm.py`, `plot_escalera.py` | Escalera nanométrica y su figura |
| `tools/prueba_velocidad_acoplado.py` | Velocidades 2–5 mm/s |
| `informe/scripts/resumen_escaleras.py` | Tabla consolidada de escaleras (para este informe) |

---

## 5.6 Cronología

| Fecha | Qué |
|---|---|
| 11/8 – 18/8 | Principio del MT6835; simulaciones del campo (magpylib); elección de imanes Ø3 × 5 mm y de la posición del sensor |
| 21/8 | Primer circuito en protoboard (Pico + TB6612), motor girando con MicroPython |
| 25/8 – 4/9 | Diseño de la rueda circular y del soporte del encoder en Onshape; ensamble del eje |
| 12/9 | Llegan imanes y encoder; ajuste del radio para un número par de imanes; tapa a presión; arquitectura del firmware |
| Antes del 29/9 | Puesta en marcha del firmware adaptado: varios errores corregidos (pin PWM de la bobina B, chequeos de calibración comentados, cálculo del RMS); calibración rearmada (una vuelta, mínimos cuadrados, Thomas) |
| 29/9 | Encoder montado girado 90°: se reorienta. Barridos de caracterización sanos (30 períodos, sin avisos). Rueda pegada al eje |
| 2/10 | Primer movimiento en lazo cerrado. Husillo medido: 5 mm (no 2). Homing más robusto |
| 9/10 | Conmutación desfasada por el homing (un período = 240° eléctricos): verificación y corrección automática |
| 13/10 | La vuelta hacia home se trababa: rampa de conmutación por el reparto de la costura → calibración de 400° plegada. `POS_KI` 30 000 → 3 000 |
| 16/10 | Guardado en flash; telemetría de 28 bytes; diagnóstico por plegado; B1 + A1 (14,1 → 1,2 µm RMS). Primera versión en git |
| 20/10 | `POS_KI` → 100. Filtro de armónicos del motor y anti-cogging aprendido (→ 1,0 µm RMS en el carril completo). Interfaz gráfica |
| 23/10 | Posición sin el piso del `float` (double + enteros): reposo 1,4 nm, pasos de 10 nm |
| 27/10 | Versión estable `v2026-10-27` |
| 30/10 – 3/11 | Motor desacoplado: techo de velocidad, torque, atraso de la corriente, fijación de la rueda, PID con rotor libre |
| 3/11 | Motor reacoplado: oscilación de 214 Hz en reposo (27 nm) |
| 6/11 | Causa: `VEL_KP`; versión final con 0,12 / 54 y calibración nueva. 11 escaleras de 10 nm |

---

## 5.7 Datos sin analizar o parcialmente analizados

- `mediciones/2026-11-03/barrido_pid_*` (con el rotor libre): solo `.npz` en esta copia; el resumen está en la rama `prueba/sin-acople` (`mediciones/2026-11-03/barrido_pid_*/resumen.txt`).
- `mediciones/2026-11-06/escalera_110414.csv` y `escalera_110729.csv`: ráfagas de ~25 Hz de hasta ±250 nm.
- `mediciones/2026-11-06/traza_corte_tabla_flash.csv`: traza del corte con la tabla de flash.
- `mediciones/2026-10-30/` (rama `prueba/sin-acople`): torque y velocidad a lazo abierto, con gráficos.
