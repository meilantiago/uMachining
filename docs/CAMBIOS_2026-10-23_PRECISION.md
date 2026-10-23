# Cambios del 23/10/2026: posición sin el piso del float

Rama `precision-nanometrica`.

## Problema

En la interfaz, con el carro quieto, la posición y el error de seguimiento se veían en escalones de unos 3 nm a 30 mm. Cerca de 255 mm los escalones llegaban a unos 24 nm. No era un problema del gráfico. La posición recorría todo el firmware como `float` de 32 bits, que tiene 24 bits de mantisa:

| Dónde | Paso del float |
|---|---|
| Entrada de la tabla del encoder (`evaluate(float(raw))`, unos 63 millones de cuentas por vuelta) | 2,5 nm a 30 mm; 20 nm por encima de unos 170 mm |
| `motor_pos` en rad | 3 nm a 30 mm; 24 nm de 204 a 255 mm |
| Objetivo: `strtof` del G1, perfil, `target_position_mm` | 15 nm de 128 a 255 mm |
| Telemetría (objetivo y posición en `float` [mm]) | 15 nm de 128 a 255 mm |

El PID calculaba su error con esa resolución. Cerca de 255 mm el lazo, en la práctica, tenía un sensor de 24 nm. El encoder rinde unos 0,08 nm por cuenta.

En la grabación `mediciones/2026-10-20/interfaz_20261020_175830.csv`, con el objetivo fijo en 30,1579 mm, la posición saltaba entre 7 niveles separados unos 3 nm, con un desvío de 2,8 nm.

## Cambios

1. **Tabla del encoder** (`LookupTable::evaluate_periodic`): las vueltas enteras se sacan con aritmética entera de 64 bits antes de pasar a float. La tabla recibe solo la posición dentro de una vuelta, menos de 2^26 cuentas. El resultado se arma en double: valor de la tabla + vueltas × 2π. En una simulación con una tabla sintética, el error de evaluación bajó de 24 nm a 0,65 nm en todo el recorrido. No se usa la división de 64 bits, que en el M0+ es lenta: el cociente se estima en float y se corrige con enteros.
2. **Servo**: `motor_pos` es double y `pos_error = float(objetivo − posición)` se calcula en double. El resultado es chico y entra en float sin pérdida. La conmutación sigue en float: 0,002 rad eléctricos a 320 rad de motor alcanzan.
3. **Objetivo en double**: el parser de G-code (`strtod`, `get_value_double`), el perfil trapezoidal (posiciones y tiempo transcurrido; en float, a los 100 s de movimiento el tiempo iba de a 8 µs, 15 nm a 2 mm/s), `SharedData::target_position_mm` y `AxisController::update_servo`.
4. **Entre núcleos**: un double no se puede leer desde el otro núcleo sin riesgo de leer una mitad vieja y una nueva.
   - El núcleo 1 publica la posición como `int32` en unidades de 0,25 nm (`POSITION_LSB_MM`, ±536 mm), que se escribe en una sola operación.
   - El objetivo double se escribe siempre con el spinlock tomado. Las escrituras desde comandos (M17, G28, M56, M57) pasan por `SharedData::set_target_position_mm`.
5. **Telemetría**: objetivo y posición viajan como `int32` en unidades de 0,25 nm. El paquete sigue teniendo 28 bytes. El encabezado cambia de `0xAA 0x55` a `0xAA 0x56`, para que un script viejo no lea esos enteros como float: simplemente no recibe telemetría. Se actualizaron `tools/interfaz.py` y `tools/closed_loop_test.py`. `pid_sweep.py`, `ki_sweep.py` y `cogging_learn.py` lo usan a través de este último.
6. **Aprendizaje del anti-cogging**: la resta objetivo − posición se hace en double.
7. **Interfaz**: destino, G1 e indicadores con 5 decimales (10 nm). Error promedio con 1 nm de resolución. CSV con posición `%.7f` mm y error `%.4f` µm.
8. M114 y M119 informan la posición con 6 decimales.

## Lo que no cambia

- El corrector de cascada (apagado por defecto) sigue en float.
- La traza M121 y la tabla de campo siguen en float. Son diagnósticos o conmutación.
- La interfaz sigue mostrando el error con la misma convención: objetivo del perfil en el tick de 2 kHz menos la última posición del núcleo 1.

## Para probar en el riel

- Que el lazo del núcleo 1 no se haya hecho más lento: columna `loops` de M121, iteraciones por ms. El double por software agrega unas 10 operaciones por vuelta.
- Con el carro quieto a 30 mm y a 200 mm: el error ya no debería verse en escalones de 3 o 24 nm.
- Movimientos de 10 nm (`G1 X…`) a 30 mm y a 200 mm.
