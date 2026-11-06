# 9. Inconsistencias y datos a confirmar

**Criterio:** cuando el cuaderno o los archivos de sesión no coinciden con el firmware, manda el firmware de la versión fija (06/11/2026, `src/` en `9b913bc`). Lo que se resuelve así ya no figura como inconsistencia: está en la sección 9.1 con el valor que hay que usar, para corregir el texto del cuaderno al pasarlo al informe.

## 9.1 Valores fijados por el firmware del 06/11 (usar estos)

| Dato | Valor del firmware | Dónde está | Qué decía el cuaderno o la sesión |
|---|---|---|---|
| Paso del husillo | **5 mm/vuelta** | `hw_config.h`: `LEAD_SCREW_PITCH_MM` | 2 mm (`DOCUMENTACION_ENCODER_Y_CALIBRACION.md`, `SESION_2026-09-29.md`); sus µm se multiplican por 2,5 |
| Frecuencia del lazo cerrado (núcleo 1) | **~16 kHz** (15,9 vueltas/ms, 62,8 µs por vuelta) | Medido con `M121` sobre el código del lazo de `8c9fc0b`; desde entonces `src/` solo cambió en `hw_config.h` (ganancias) | ~22 kHz (cuaderno 12/9 y 29/9) |
| Reloj del sistema | 250 MHz | `main.cpp`: `set_sys_clock_khz(250000, …)` | — |
| Perfil y telemetría | Perfil en interrupción de 2 kHz; telemetría a 1 kHz con `M120 S1 P2` | `main.cpp`, `shared_data.h` | idem |
| Paquete de telemetría | **0xAA 0x56 + 28 bytes**; cola circular de 1000 puntos | `shared_data.h`, `main.cpp` | 18 bytes (cuaderno 12/9) |
| Adelanto del campo | Salida del PI de velocidad (más el anti-cogging), **limitada a ±0,45π = ±81° eléctricos** | `servo_controller.cpp` | "Siempre 90° adelante del rotor" (cuaderno 12/9) |
| Barrido de calibración | **400° por sentido**, descartando 15° en cada punta | `hw_config.h`: `CALIBRATION_RANGE`, `CALIBRATION_TRANSIENT_SKIP` | 370° (cuaderno, puesta en marcha) |
| Residuo de la tabla | 0,035° de motor = 0,48 µm RMS (calibración de hoy con este firmware) | `mediciones/2026-11-06/m56.log` | "Sin errores de interpolación" (cuaderno 12/9) |
| Ruido en reposo | **1,43 nm** (mediana de 11 corridas, versión de hoy) | `informe/scripts/resumen_escaleras.py` | "0,002 µm" (README de esta rama, `ELECCION_POS_KI.md`): medido con el firmware anterior al 23/10, que guardaba la posición en `float` con pasos de ~3 nm. No usar |
| Error del barrido del encoder | 4,4–5,2 µm RMS (con 5 mm) | idem paso del husillo | "~2 µm RMS" (con 2 mm) |
| Encoder | SPI a 8 MHz, BW 5, histéresis 4, CRC desactivado | `main.cpp`, `axis_controller.cpp`, `hw_config.h` | — |
| Corte por falla | \|error\| > 1 mm durante 0,5 s | `axis_controller.cpp` | — |

## 9.2 Inconsistencias que no resuelve el firmware

| Tema | Una versión | Otra versión | Propuesta |
|---|---|---|---|
| Grado del imán en la comparación | N38 (cuaderno 18/8, en dos lugares) | N35 (título de la misma sección; imanes comprados; README) | N35 |
| Óptimo del sensor | Y = 2–3 mm, Z = 4,2–4,5 mm, ~30 mT (tabla del 14/8) | Y = 1,59 mm, Z = 4,05 mm, 58 mT (18/8; notebook de `origin/master`) | El del 18/8; la tabla del 14/8 era con la región restringida |

## 9.3 Datos que faltan o hay que confirmar

1. **Tensión de la fuente** en las mediciones finales (10 V o 12 V, con factor 0,6). Cambia el torque disponible y la corriente. Con 33 Ω por fase y 12 V × 0,6, la corriente quieta sería ~0,22 A.
2. **Modelo del motor y del husillo** (fabricante, NEMA, diámetro del husillo, precarga de la tuerca, tipo de acople). R, L y K_t salen de `PLAN_DE_PRUEBAS_MODULAR.md`: confirmar de dónde salieron (¿medidos? ¿hoja de datos?).
3. **Material de la rueda** (PLA, PETG) y si se cambiaron los tornillos de acero cerca del sensor (recomendado en `DOCUMENTACION_ENCODER…` §8).
4. **Distancia real del chip a los imanes** en el montaje actual. El borrador de publicación dice "voladizo de 56 mm, ~0,3 mm de juego radial, campo cerca de 30 mT", pero no hay medición documentada.
5. **Reproducibilidad de la calibración**: no se compararon varias `M56` seguidas. Bastaría con 3–5 calibraciones, comparando RMS, atraso, desfase y la tabla.
6. **Carril completo con la versión final**: solo hay 0–50 mm. Correr `closed_loop_test.py` 0 → 255 → 0 mm.
7. **Escaleras en otras posiciones** con la versión final (10, 100, 200 mm). Solo hay 30 mm.
8. **Stick-slip con las ganancias finales**: mirar las trazas de las escaleras del 06/11 (el `plot_escalera.py` suaviza 50 ms; mirar también la telemetría cruda) y decir si aparece.
9. **Ráfagas de ~25 Hz** del 06/11 11:04 y 11:07 (hasta ±250 nm). ¿Coinciden con algo en el laboratorio? Si no se explican, mencionarlas como observación abierta.
10. **Velocidades > 2 mm/s con las ganancias finales**: no medido.
11. **Incertidumbres**: el Informe 3 da cada resultado con su incertidumbre. Acá las cifras son estadísticos de series temporales. Para el informe, como mínimo: desvío estándar y N de cada estadístico, y error estándar de la media donde corresponda (por ejemplo, salto medio 10,02 ± 0,06 nm como error estándar con N = 55; el desvío de 0,41 nm es la dispersión de cada salto).
12. **Interferómetro**: el título del proyecto lo incluye. Definir con los directores cómo presentarlo (trabajo futuro, o un ensayo mínimo con comparador antes de entregar).

## 9.4 Riesgos de redacción

- No decir "precisión de 10 nm" ni "exactitud de 1 µm": decir **resolución** y **error de seguimiento medido por el encoder**.
- No copiar "precisión sub-milimétrica" del cuaderno (12/9) ni de `DOCUMENTACION_ARQUITECTURA.md`: el eje trabaja en µm y nm.
- No atribuir la ondulación de 83 µm o la de 25 µm a "movimiento real del carro" como hecho: es la explicación más consistente, no verificada con un sensor externo.
- Separar lo heredado del proyecto original de lo propio (ver 02 §2.4), y citar el repositorio de origen.
