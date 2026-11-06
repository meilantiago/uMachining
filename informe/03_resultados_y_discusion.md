# 3. Resultados y discusión

Resultados ordenados en el orden lógico del informe (sensor → calibración → lazo en movimiento → reposo y resolución → velocidad), no en orden cronológico. Cada bloque tiene los números, de dónde salen, cómo se interpretan y qué figura usar (ver [07_figuras_y_datos.md](07_figuras_y_datos.md)).

Convenciones: las longitudes en µm o nm son **de carro**, con husillo de 5 mm. Para pasar unidades, ver [05_apendice.md](05_apendice.md) (1° de encoder = 0,463 µm; 1° de motor = 13,9 µm; 1° eléctrico = 0,278 µm).

> **Ojo con documentos viejos.** `docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md` y `docs/SESION_2026-09-29.md` convierten con husillo de 2 mm: sus µm hay que multiplicarlos por 2,5. Acá ya están convertidos.

---

## 3.1 El sensor: no linealidad y ruido

### Barridos de caracterización (lazo abierto, encoder solo mide)

| Corrida | Condición | Campo débil / overspeed / CRC / saltos | Períodos por vuelta | No linealidad RMS ida / vuelta | Máx. |
|---|---|---|---|---|---|
| 29/09 17:36–18:04 | Sensor mal orientado (plano XY) | 10–18 % / 701–2092 / — / 317–528 | 15–22 (inválido) | 65–103° enc | — |
| 29/09 18:21 | Sensor reorientado | 0 / 0 / 0 / 0 | 29,98 | 10,4 / 11,2° enc = 0,35 / 0,37° motor = **4,8 / 5,2 µm** | 33° enc = 15 µm |
| 03/11 15:34 | Motor sin acople | 0 / 0 / 0 / 0 | 30,00 | 9,6 / 10,0° enc = 0,32 / 0,33° motor = **4,4 / 4,6 µm** | — |
| 03/11 17:04 | Motor reacoplado | 0 / 0 / 0 / 0 | 30,00 | 10,6 / 10,9° enc = 0,35 / 0,36° motor = **4,9 / 5,0 µm** | 38° enc = 18 µm |

Fuentes: `docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md` §4.10–4.11; `git show prueba/sin-acople:mediciones/2026-11-03/RESULTADOS.md`; la última corrida se resumió para este informe con `tools/encoder_sweep.py --from-csv mediciones/2026-11-03/encoder_sweep_20261103_170456.csv`.

**Forma del error.** Ondulación suave de ±10–20° de encoder por período, repetible entre vueltas y entre ida y vuelta. Por sectores de 4 imanes va de ~6° (mejores) a 12–18° (peores), una modulación de una vez por vuelta compatible con excentricidad de la rueda o diferencias entre imanes (`plot_periods.py`, figura `*_periodos_2.png`).

**Discusión.**
- Con el sensor bien orientado no hay avisos del chip y el número de períodos por vuelta es el entero esperado. El diagnóstico "**si los períodos por vuelta no dan un entero cercano a 30, el problema es de montaje y no se arregla en el firmware**" es una de las conclusiones prácticas del trabajo.
- La no linealidad (~5 µm RMS, ~15–18 µm máx.) es estable en el tiempo (29/09 a 03/11, desmontes incluidos) y **repetible**, que es la condición para corregirla con la tabla.
- El error se mide contra el campo comandado, así que incluye el atraso variable del rotor y las irregularidades del micropaso: **sobreestima** la no linealidad propia del sensor.
- Comparación con la simulación: el modelo predice un ángulo casi perfectamente lineal en el óptimo; lo medido (±10–20° de encoder) indica que el montaje real no está en el óptimo (voladizo de 56 mm, juego radial ~0,3 mm, campo cerca de 30 mT; ver 09).

### Ruido del encoder (06/11)

- **Motor apagado:** 0,6–0,9 nm muestra a muestra (equivalente a ~8–11 cuentas), sin bits de estado, sin nada entre 150 y 260 Hz.
- Con el rotor libre aparece una vibración de ~3 nm a 25 Hz, mecánica o ambiental; con el lazo cerrado baja a < 1 nm.

Fuente: `mediciones/2026-11-06/RESULTADOS.md` §1. **El piso de ruido del sensor (~1 nm) es del orden del ruido en reposo con el lazo cerrado (~1,4 nm, sección 3.6)**: el lazo no agrega casi nada en reposo.

---

## 3.2 La calibración

### Error de conmutación a lo largo de la vuelta (13/10)

`M57` separó, a lo largo de cada vuelta, el error de la tabla (promedio de los dos sentidos) del atraso por fricción (semidiferencia):

| Versión de la calibración | Error de conmutación dentro de la vuelta | Salida media en lazo cerrado (ida / vuelta) |
|---|---|---|
| Repartiendo el "desfase de la costura" | rampa de −2° a −36° eléctricos | +23° / −59° (la vuelta tocaba el límite de 81° y cortaba) |
| Final (400°, sin puntas, plegado) | entre −4° y +4° | +43° / −42° |

El "desfase en la costura" (~0,6° de motor) **no existía**: en los datos crudos una vuelta del encoder daba 360,00° de motor en cada sentido (±0,1°). Era un artefacto de evaluar la tabla en los bordes, donde solo hay muestras de un sentido y el atraso del rotor no se cancela. Fuente: cuaderno 13/10; `docs/SESION_2026-10-13.md`; `mediciones/2026-10-13/diagnostico_vuelta/`.

**Discusión.** Es un buen ejemplo del método: un síntoma (el carro se traba solo hacia el home, siempre en la misma fase de la vuelta) se separó en dos causas con una medición diseñada para eso, y la corrección se verificó con la misma medición.

### Calibración de la versión final (06/11, `mediciones/2026-11-06/m56.log`)

| Magnitud | Valor |
|---|---|
| Homing | tope tras 1,36 vueltas |
| Períodos de encoder por vuelta | 30,013 |
| RMS del ajuste de la tabla | 0,035° de motor = **0,48 µm** |
| Atraso del rotor (lazo abierto, 0,32 mm/s) | −0,36° de motor = 5,0 µm |
| Salto en la costura | 0,023° de motor = 0,32 µm |
| Huella del motor sacada de la tabla | 200/vuelta (paso): 1,20 µm · 400: 0,64 µm · 100: 0,40 µm · 50: 0,39 µm · 350: 0,15 µm · resto < 0,07 µm |
| Desfase de conmutación | −5,2° eléctricos (con el contador corrido ±1 período: −130° / +108°) |
| Anti-cogging reaprendido | RMS 0,90 → 0,87 µm |

Calibraciones anteriores: RMS de la tabla 0,111° (13/10, tras remontar la rueda), 0,088° (20/10). **Falta** comparar varias calibraciones seguidas para tener la reproducibilidad de la tabla (ver 04).

### Tabla guardada en flash contra calibración nueva

| Fecha | Tabla de flash (después de `G28`) | Calibración nueva |
|---|---|---|
| 16/10 | `M57`: +7° a +23° en toda la vuelta; dos cortes en los primeros 20 mm | ~+9° |
| 06/11 | `M57 X5`: **+73 ± 18°** (+26 a +142°); `G1 X30` cortó con la salida saturada | `M57`: **+30,2 ± 3,7°** (igual que el 03/11) |

(Los valores de `M57` en un solo sentido incluyen el atraso por fricción, ~+30°; lo que importa es la comparación.) Fuente: `mediciones/2026-11-06/RESULTADOS.md` §2, `m57_tabla_flash.log`, `m57_postcal.log`.

**Discusión.** La tabla del 06/11 en flash era anterior al reacople del motor. La conmutación es sensible a cambios mecánicos y a dónde cae el homing dentro del período; la verificación del homing la mide junto al home y no ve errores en el resto de la vuelta. **Conclusión operativa: correr `M56` en cada arranque y después de cualquier cambio mecánico.**

---

## 3.3 Error de seguimiento en movimiento: de 14 a 1 µm

### Diagnóstico inicial (16/10, carril completo 0 → 255 → 0 mm a 2 mm/s, dos veces)

504 104 muestras, 128,1 s por tramo. Presupuesto por plegado (`docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md` §6):

| Componente | Ida | Vuelta | Origen identificado |
|---|---|---|---|
| Constante (atraso) | 13,25 µm | 13,24 µm | Límite de la integral del lazo de posición (ec. 1.7: predice 12,6 µm) |
| Por imán (83 µm) | ±4,5 µm | ±4,2 µm | Velocidad calculada con las cuentas crudas, sin la tabla |
| Por vuelta (5 mm) | ±2,2 µm | ±1,9 µm | Diferencias entre imanes y sector de fricción a 2,3–3,2 mm de cada vuelta |
| Por paso (25 µm) | ±0,4 µm | ±0,6 µm | Motor |
| Por ciclo eléctrico (100 µm) | ±0,4 µm | ±0,4 µm | Motor |
| Resto (desvío) | 1,4 µm | 1,3 µm | Ruido y picos de saturación (máx. 23,8 µm) |

Las dos idas coinciden en centésimas de µm: el comportamiento es completamente repetible.

**Mecanismo de la ondulación por imán.** El ángulo crudo tiene ±3,8 µm de no linealidad por imán. Si la velocidad se calcula con cuentas crudas, el lazo "ve" una velocidad que oscila aunque el motor gire parejo, la corrige y hace oscilar al motor de verdad. Evidencia: la velocidad que veía el lazo ondulaba 0,69 mm/s pico a pico y la real (según la tabla) 1,53 mm/s, ±38 %. Si la oscilación fuera real y el lazo la estuviera corrigiendo, sería al revés. Orden de magnitud: ±2π·3,8/83 ≈ ±29 %.

### Experimento de intervención (16/10, misma calibración, carril completo)

B1 = velocidad desde la pendiente local de la tabla. A1 = realimentación anticipada de la velocidad del perfil.

| Configuración | Atraso | Por imán | RMS | Máx. ida / vuelta | Muestras saturadas (ida) |
|---|---|---|---|---|---|
| Ninguna | 13,7 µm | ±5,2 µm | 14,1 µm | 25,3 / 22,7 µm | 12 217 |
| Solo B1 | 13,3 µm | **±0,2 µm** | 13,3 µm | 16,7 / 18,6 µm | 53 |
| Solo A1 | **0,7 µm** | ±5,6 µm | 4,0 µm | 12,0 / 12,0 µm | 5 358 |
| **B1 + A1** | **0,7 µm** | **±0,2 µm** | **1,2 µm** | **4,0 / 5,9 µm** | **3** |

Fuente: cuaderno 16/10; `mediciones/2026-10-16/lazo_cerrado_20261016_10*_V?F?`; figura `reporte/fig6_comparacion.png`.

**Discusión.**
- Cada cambio elimina **su** componente sin tocar la otra: confirma los dos diagnósticos por intervención, no solo por correlación.
- Los 0,7 µm de atraso que quedan son el desfasaje de la telemetría (0,5 ms a 2 mm/s; +0,7 en la ida, −0,7 en la vuelta), no del carro.
- Arranques y frenadas: error máximo de 23,3 a 4,9 µm. Reposo después de llegar: 0,3–0,4 µm.
- Efecto colateral: la componente del paso (25 µm) subió de 0,6 a 2,1 µm en la vuelta, porque la tabla, armada en lazo abierto, guarda la huella del micropaso, y con B1 esa huella pasa a la velocidad. Es el punto de partida de 3.4.

### Huella del motor: filtro de armónicos y anti-cogging (20/10)

| Carril completo 0 → 255 → 0 mm a 2 mm/s | RMS ida / vuelta | Máx. |
|---|---|---|
| Antes (B1 + A1, `POS_KI` 100) | 1,78–1,83 µm | 4,8–6,5 µm |
| Solo filtro de armónicos | 1,6 / 1,6 µm | 6,8 µm |
| **Filtro + anti-cogging aprendido** | **1,0 / 1,0 µm** | 6,7 µm |

Aprendizaje (`cogging_153911_dos_tablas`, ganancia 0,8, 0 → 50 → 0 mm):

| Iteración | RMS | Paso (25 µm) | Ciclo (100 µm) | Corrección del PI por ciclo |
|---|---|---|---|---|
| 0, referencia | 1,57 µm | 1,48 µm | 0,55 µm | 20,0° pp |
| 1, prueba (+0,5 × patrón) | 2,17 µm | 2,36 µm | 0,36 µm | 28,6° pp |
| 2 | 1,11 µm | 0,60 µm | 0,21 µm | 4,8° pp |
| 3 | 0,99 µm | 0,07 µm | 0,17 µm | 1,8° pp |
| 4–5 | 0,99 µm | 0,06 µm | 0,12–0,14 µm | 1,7–1,9° pp |

Fuente: `docs/CAMBIOS_CALIBRACION_2026-10-20.md`; `mediciones/2026-10-20/cogging_*`.

**Discusión.**
- Copiar directamente el patrón del PI a la tabla **empeoró** el error (1,58 → 2,84 µm): a 2 mm/s el ciclo eléctrico cae en 20 Hz y el paso en 80 Hz, y a 40–80 Hz el lazo responde con ~180° de atraso (medido: +177° en el armónico 2). La tabla empujaba a favor de la perturbación. Hizo falta medir la respuesta de cada armónico (método de la secante) y corregir con esa respuesta: es control por aprendizaje iterativo.
- Con una sola tabla el aprendizaje se estancó en 1,27 µm: en la vuelta el patrón pasa con el tiempo invertido y la respuesta es la conjugada. Se usan dos tablas (parte par e impar en el sentido).
- La tabla depende de la calibración (con tablas viejas y calibración nueva: 1,4 / 1,6 µm en lugar de 1,1 / 1,1), pero las respuestas armónicas no: por eso alcanza con dos pasadas automáticas al final de cada `M56`.
- A 4 mm/s la tabla aprendida a 2 mm/s empeora el error (RMS 3,3 / 2,9 → 4,8 / 7,7 µm), porque la respuesta del lazo cambia con la frecuencia. Se desvanece entre 2 y 3 mm/s.
- **Abierto:** no se sabe si la componente de 25 µm era movimiento real del carro o error de la tabla. El filtro saca la parte de medición; el anti-cogging corrige la parte de movimiento. Solo un sensor externo rápido lo distingue.

### Versión final (06/11, después de `M56`)

0 → 50 → 0 mm a 2 mm/s (`mediciones/2026-11-06/lazo_cerrado_20261106_093001_postcal_kp012_resumen.txt`):

| Tramo | RMS | Máx. | Salida media / máx. | > 70° | Encoder |
|---|---|---|---|---|---|
| Ida | **0,9 µm** | 3,0 µm | +33,4° / 44,0° | 0 % | sin avisos |
| Vuelta | **1,0 µm** | 3,5 µm | −34,8° / 45,7° | 0 % | sin avisos |

El 03/11 con las ganancias anteriores (`VEL_KP` 0,2 / `VEL_KI` 90): 1,7 / 1,7 µm RMS, 4,5 / 6,3 µm máx., salida máx. 54–62°.

> Falta el carril completo con la versión final. El último carril completo es del 20/10 (1,0 µm RMS con filtro + anti-cogging, ganancias 0,2 / 90).

---

## 3.4 Ganancia integral de posición (`POS_KI`)

| `POS_KI` | Reposo | Movimiento a 2 mm/s | Fuente |
|---|---|---|---|
| 30 000 (heredado) | Ciclo límite de 40–50 Hz, ±17 µm, 32 µm pp en X0 | ~11 µm | cuaderno 13/10 |
| 3 000 | ~1 µm pp; con B1+A1: caza ±0,45 µm a 4,6 Hz | igual | cuaderno 13/10; `barrido_ki_123236` |
| 1 500 | Caza permanente ±0,17 µm a 2,7 Hz (desvío 0,13–0,17 µm) | 1,76 µm RMS | `barrido_pid_144225` |
| 250 | Caza permanente ±0,05 µm a 1–2 Hz | 1,78 µm | idem |
| 200 | Oscilación amortiguada ±0,03 µm | 1,78 µm | idem |
| 150 | Quieto, "chasquidos" aislados de ±0,15 µm | 1,78 µm | idem |
| **100** | **Quieto, desvío 0,001–0,002 µm**; llega a 0 en ~0,5 s | **1,79 µm** | idem |
| 0 | Quieto, 0,002 µm, pero el atraso deriva +0,2 µm después de 130 mm | 1,79 µm | idem; `barrido_pid_140929` |

Verificación en el carril completo (`barrido_pid_145957`): con 100, RMS 1,78 µm (contra 1,76–1,77 con 1500), atraso plano a lo largo del eje (rango 0,012 µm), quieto en las puntas (0,006–0,017 µm contra 0,12–0,18 µm oscilando).

**Discusión.**
- **El movimiento no depende de `POS_KI`**: con la realimentación anticipada, la velocidad la pone el perfil. La integral de velocidad ya anula el error final contra una carga constante. La de posición solo corrige desajustes lentos proporcionales a la velocidad (la deriva de +0,2 µm con 0).
- El reposo sí: por encima de un umbral (200–500 según el día y la posición) la integral caza contra la fricción estática. Es el ciclo límite de 1.5.
- Se eligió 100: reposo igual que 0, conserva integral de posición (T_i = 0,6 s), margen ×2–2,5 bajo el umbral, que depende de la fricción.
- Los desvíos de 0,001–0,002 µm de esta tabla son del firmware anterior al 23/10, que guardaba la posición en `float` (pasos de ~2,5–3 nm a 20–30 mm). Sirven para comparar los valores de `POS_KI` entre sí, pero **no son el ruido en reposo**: con el firmware de hoy es 1,43 nm (3.6).

---

## 3.5 Reacople del motor y ganancia proporcional de velocidad (03/11 – 06/11)

Tras reacoplar el motor el 03/11, el reposo a 30 mm tenía **27 nm** de desvío (antes 1,4 nm) y los escalones de 10 nm se veían con 3–18 nm de desvío. Era una oscilación de banda angosta de ~205–214 Hz, de amplitud constante dentro de cada escalón, con correlación −0,99 entre posición y salida del lazo.

1. **No era el encoder**: con el motor apagado, 0,6–0,9 nm y nada en 150–260 Hz; la configuración del chip no había cambiado.
2. **Barrido de `VEL_KP` / `VEL_KI` en reposo** (`mediciones/2026-11-06/reposo_ganancias/`):

| `VEL_KP` | `VEL_KI` | Desvío en reposo | Potencia en 150–260 Hz |
|---|---|---|---|
| **0,12** | **54** | **2,0–2,3 nm** | 23 % |
| 0,12 | 90 | 2,8 nm | 15 % |
| 0,16 | 54 | 5,1 nm | 77 % |
| 0,16 | 72 | 9,5 nm | 91 % |
| 0,20 | 54 | 10,6 nm | 93 % |
| 0,20 | 90 (anterior) | 10,8–11,1 nm | 92–96 % |
| 0,24 | 54 | 11,7 nm | 94 % |

A 2 kHz de telemetría el pico sigue en 214 Hz: no es alias.

**Discusión.** El umbral está entre `VEL_KP` 0,12 y 0,16: con el motor reacoplado, el lazo de velocidad anterior (0,2 / 90) quedaba al borde de la inestabilidad en ~214 Hz. `VEL_KI` casi no influye. La causa más probable es un cambio en la rigidez o la fricción del acople al remontar, que movió la resonancia. Con 0,12 / 54, además, el seguimiento mejoró de 1,7 a 0,9–1,0 µm RMS y la salida máxima bajó de 62° a 46°. Lección: **las ganancias del lazo de velocidad dependen de la mecánica y hay que revisarlas después de cada desmonte** (igual que la calibración).

Contraste con el rotor libre (03/11, sin acople): el óptimo era `VEL_KP` 0,3 / `VEL_KI` 20 (0,92 µm RMS a 2 mm/s). Sin la inercia del carro la planta es otra.

---

## 3.6 Reposo y resolución: escalones de 10 nm

### Escalera a 30 mm (06/11, versión final)

11 corridas (09:27, 09:42, 10:55 y 8 seguidas de 11:03 a 11:09). Cada una: reposo de 5–10 s y 5 pasos absolutos de 10 nm de 4 s. Telemetría a 1 kHz, se descarta el primer segundo de cada escalón. Tabla completa: `python3 informe/scripts/resumen_escaleras.py mediciones/2026-11-06/escalera_*.csv`.

| Magnitud | Valor |
|---|---|
| Desvío en reposo | mediana **1,43 nm**, rango 0,79–2,94 nm (11 corridas) |
| Saltos medidos (pedidos: 10 nm) | **N = 55, media +10,02 nm, desvío 0,41 nm**, rango +8,8 a +11,3 nm |
| Desvío dentro de cada escalón | 1,0–2,9 nm en 50 de 55; 3,2–4,9 nm en 3; dos ráfagas (abajo) |

Corrida de ejemplo para la figura (09:42, `escalera_094219_claro.png`): reposo 1,30 nm; saltos +10,3, +9,8, +10,1, +9,9, +10,1 nm.

**Anomalías** (`git show 9b913bc`):
- 11:04 (escalón 4) y 11:07 (escalón 2): ráfaga de ~25 Hz de hasta ±250 nm que se apaga en menos de 1 s (desvíos 14,8 y 39,5 nm). No es la oscilación de 214 Hz. Coincide en frecuencia con la vibración mecánica de 25 Hz vista con el rotor libre (3.1). **Sin analizar.**
- 10:55: escalones 3 y 4 con 4,6–4,9 nm de desvío.

Comparación: el 03/11 con `VEL_KP` 0,2 la misma prueba dio 27 nm de desvío en reposo; los saltos medios igual daban ~10 nm (los promedios de 3 s filtran la oscilación).

**Discusión.**
- El lazo **resuelve pasos de 10 nm** (unas 125 cuentas del encoder) con una dispersión de 0,4 nm en el tamaño del salto. Que la media dé 10,02 nm es esperable por construcción: la posición la mide el mismo encoder que cierra el lazo. Lo que el resultado demuestra es **resolución y estabilidad del lazo**, no que el carro se mueva 10 nm (para eso, referencia externa).
- El ruido en reposo (~1,4 nm) es del orden del ruido del sensor con el motor apagado (0,6–0,9 nm) más la vibración ambiental: el lazo está cerca del piso que impone el sensor.
- 1,4 nm de carro equivalen a 1,0·10⁻⁴ ° de motor y 5·10⁻³ ° eléctricos: muy por debajo de la cuantización del PWM (~2,6 nm por nivel según una estimación del 01/10 con husillo de 2 mm; ×2,5 con 5 mm ≈ 6,5 nm, **a verificar**). Que el reposo sea mejor que un nivel de PWM sugiere que el lazo promedia entre niveles. No se estudió.
- **Stick-slip.** El 23/10 (ganancias 0,2 / 90), después de pasos chicos la salida se cargaba ~2 s, el carro saltaba 40–80 nm y se amortiguaba en ~8 s. En las escaleras del 06/11 los saltos aparecen limpios en el promedio y no se reporta ese ciclo. **Conviene revisar las trazas del 06/11 antes de afirmar que desapareció** (ver 09).
- Solo se midió a 30 mm. El 23/10 se vieron pasos de 10 nm también a 10 y 200 mm (+10, +11, +9 nm) y 0,8 nm de desvío a 200 mm, con las ganancias anteriores.

### Piso numérico (23/10)

| | `float` (antes) | `double` + enteros (después) |
|---|---|---|
| Reposo a 30 mm, 12 s | escalones de ~3 nm (7 niveles, desvío 2,8 nm) | desvío 1,4 nm, niveles de 0,25 nm |
| Reposo a 200 mm, 3 s | escalones de 12–24 nm (teórico) | desvío 0,8 nm |
| Pasos de 10 nm a 10, 30 y 200 mm | no se resuelven cerca del final | +10, +11, +9 nm |
| Vueltas del lazo por ms | 16,0 | 15,9 |

Fuente: `docs/CAMBIOS_2026-10-23_PRECISION.md`.

---

## 3.7 Velocidad y margen de torque

### Con el carro (motor acoplado)

0 → 50 → 0 mm el 03/11, ganancias **anteriores** (0,2 / 90), factor 0,6 (`mediciones/2026-11-03/lazo_cerrado_*_acoplado_v*`):

| Velocidad | RMS ida / vuelta | Máx. | Salida media | Tiempo con salida > 70° |
|---|---|---|---|---|
| 2 mm/s | 1,7 / 1,7 µm | 6,3 µm | ±34° | 0 % |
| 3 mm/s | 3,5 / 2,1 µm | 9,5 µm | ±44° | 1–9 % |
| 4 mm/s | 4,6 / 4,4 µm | 10,6 µm | ±56° | 33 % |
| 5 mm/s | 3,6 / 3,4 µm | 9,7 µm | ±66° | 39–40 % |

Ninguna corrida cortó. El 20/10, a 4 mm/s: 3,3–4,5 µm RMS, máx. 21–30 µm, salida > 70° el 35–42 % del tiempo.

**Discusión.** A partir de ~3 mm/s el motor se queda sin margen de torque con factor 0,6: la salida pasa buena parte del tiempo cerca del límite de 81°. **No es un problema de calibración ni de ganancias sino de torque disponible.** Según el 13/10, la fricción ya consume más del 60 % del torque a corriente 0,6, con un pico una vez por vuelta (acople o alineación del husillo). Con las ganancias finales solo se midió 2 mm/s.

### Motor solo (sin acople, 30/10 – 03/11, factor 1,0, 12 V, fuente limitada a 2 A)

| Ensayo | Resultado |
|---|---|
| Lazo abierto, rampa hasta perder el paso (`M63`) | ~33–53 mm/s según la aceleración (300 mm/s²: 45–53 mm/s); resonancia de media velocidad entre 15 y 35 mm/s |
| Lazo abierto con rampa, firmware `motor_libre`, el encoder solo mide (30/10; cuaderno, sección Firmware; commit 442d5f5) | Sigue lo comandado hasta 880 rpm (73 mm/s equivalentes) con 100 rpm/s, pero inestable: un roce le hace perder el campo. Operativo hasta 500 rpm (42 mm/s). A 300 rpm el chip marca overspeed en ~15 % de las muestras (BW 5) |
| Lazo cerrado, sentido + | 10 mm/s: 5,3 µm RMS (17 µm máx.); 15 mm/s: se satura y se clava |
| Atraso de la corriente | TB6612 en tensión: el campo llega ~2–3 ms tarde (L/R ≈ 1,1 ms nominal); a 5–10 mm/s se come el adelanto. Compensación `arctan(ωτ)` probada (`M62`) |

Fuente: commits de `prueba/sin-acople` (bc6dc06, 73ac7f4, 1c2ac7f, abba99d) y cuaderno (sección Firmware).

**Discusión.** El techo del motor solo (decenas de mm/s) está muy por encima de lo que se usa. Con el carro, el límite es la fricción de la transmisión más el torque a factor 0,6. Para ir más rápido: más tensión o corriente (el TB6612 admite 1,2 A continuos), compensar el atraso de la corriente, y revisar la alineación del acople.

### Fijación de la rueda (03/11)

Rampas violentas en lazo abierto (hasta 40 mm/s y 1000 mm/s²) con `G28` después de cada una: desfase de conmutación entre −8° y +14° eléctricos, sin acumularse; la misma dispersión que con movimientos suaves (depende de la posición en la vuelta). El 30/10, antes de ajustar la rueda, se había corrido 35–42°. **Con la rueda ajustada no se desliza.**

---

## 3.8 Evolución del desempeño (tabla resumen para el informe)

| Fecha | Cambio | Seguimiento a 2 mm/s (RMS / máx.) | Reposo |
|---|---|---|---|
| 13/10 | Calibración corregida; `POS_KI` 30 000 → 3 000 | ~11 µm atraso | 32 µm pp → ~1 µm pp |
| 16/10 | Diagnóstico (sin B1/A1) | 14,1 / 25 µm | 0,1–0,4 µm |
| 16/10 | B1 + A1 | 1,2 / 4–6 µm | 0,3–0,4 µm |
| 20/10 | `POS_KI` 1 500 → 100 | 1,78 / 5,2 µm | sin caza (posición en `float`: no resuelve menos de ~3 nm) |
| 20/10 | Filtro de armónicos + anti-cogging | 1,0 / 6,7 µm (carril completo) | — |
| 23/10 | Posición en `double` | — | 1,4 nm (30 mm), 0,8 nm (200 mm) |
| 03/11 | Reacople del motor (ganancias 0,2/90) | 1,7 / 6,3 µm (0–50 mm) | 27 nm (oscilación a 214 Hz) |
| **06/11** | **`VEL_KP` 0,12, `VEL_KI` 54 (versión final)** | **0,9–1,0 / 3,5 µm (0–50 mm)** | **1,43 nm (mediana); pasos de 10,02 ± 0,41 nm** |

Una figura con esta evolución (RMS en escala logarítmica contra fecha, anotando cada cambio) resume bien el trabajo.

---

## 3.9 Discusión general

- **Qué se logró.** Un eje de bajo costo cuyo lazo sigue la trayectoria con ~1 µm RMS a 2 mm/s, mantiene la posición medida con ~1,4 nm de desvío y resuelve pasos de 10 nm. Cada mejora tuvo un diagnóstico cuantitativo y una verificación por intervención.
- **De dónde viene la resolución.** Tres factores multiplicativos: el husillo (0,8 mm/rad), el engranaje magnético (×30) y los 21 bits del chip. Y un factor de software que había que destrabar: el piso del `float`.
- **Qué vería un instrumento externo** (de `docs/METODO_DE_ERRORES` §2). Un comparador con el carro quieto **no** vería las mejoras en movimiento: el carro queda en el mismo lugar, con el error del encoder y de la transmisión que tenía. Un sensor rápido (capacitivo, LVDT o interferómetro a ≥ 500 muestras/s) en movimiento sí vería desaparecer la ondulación de 83 µm y el atraso de 13 µm. La superficie de una pieza mecanizada con avance constante también.
- **Limitaciones de la medición.**
  1. Todo está en el marco del encoder del motor: la exactitud del carro no se midió.
  2. La "verdad" de la calibración es el campo comandado, con sus irregularidades.
  3. Sin presupuesto de incertidumbre de las cifras: las cifras son estadísticos de series temporales (desvío, RMS, media de saltos), no mediciones con incertidumbre propagada.
- **Comparación con el proyecto original** (si se quiere incluir): su README informa resoluciones del orden de 50 nm en el efector de un delta; acá, en un eje lineal, 10 nm. La comparación no es directa: la relación mm/rad es ~19 veces más favorable con el husillo.
