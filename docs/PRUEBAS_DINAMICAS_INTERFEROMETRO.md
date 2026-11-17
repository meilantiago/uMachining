# Pruebas en movimiento con interferómetro

**Versión:** 13/11/2026 (propuesta, todavía sin ejecutar).
**Dónde:** INTI, con su interferómetro. Se desconoce el modelo y la disposición óptica, así que este documento no supone ninguno.
**Alcance:** solo pruebas dinámicas. Las estáticas (exactitud de posicionamiento tipo ISO 230-2 y backlash) van en otro documento.

## 1. Para qué sirven

Todas las cifras de error que tenemos hasta hoy son **error de seguimiento contra el encoder**. El encoder está del lado del motor, así que el carro nunca se midió con una referencia independiente. El interferómetro mide eso.

Se usan tres señales y tres errores:

| Señal | Qué es |
|---|---|
| x_c(t) | posición comandada por el perfil |
| x_e(t) | posición del encoder, llevada a unidades de carro |
| x_i(t) | posición del interferómetro |

| Error | Definición | Qué dice |
|---|---|---|
| e_t | x_c − x_e | qué tan bien el lazo sigue al encoder (lo que medimos hasta ahora) |
| e_m | x_e − x_i | cuánto se aparta el carro de lo que el encoder cree: husillo, elasticidad, juego, dilatación, no linealidad del encoder |
| e | x_c − x_i = e_t + e_m | lo que importa para micromecanizar |

Como el lazo cierra sobre el encoder, un error del encoder se imprime en el carro. Lo que le pasa al carro y el encoder no ve (flexión del husillo, vibración de la guía) solo aparece en x_i.

### Frecuencias de origen espacial

Un error que depende de la posición tiene frecuencia f = v/λ, así que se mueve con la velocidad. Una resonancia mecánica queda fija en Hz. Por eso cada prueba se repite a varias velocidades.

| Origen | Período en el carro | a 1 mm/s | a 2 mm/s | a 5 mm/s |
|---|---|---|---|---|
| Vuelta del husillo (paso 5 mm) | 5 mm | 0,2 Hz | 0,4 Hz | 1 Hz |
| Paso del motor (200/vuelta) | 25 µm | 40 Hz | 80 Hz | 200 Hz |
| Imanes de la rueda (60/vuelta) | 83,3 µm | 12 Hz | 24 Hz | 60 Hz |
| Período del encoder (30/vuelta) | 166,7 µm | 6 Hz | 12 Hz | 30 Hz |

A 5 mm/s el armónico del paso cae en 200 Hz, muy cerca de la oscilación de 205–214 Hz que apareció con `VEL_KP` 0,2 tras el reacople (`mediciones/2026-11-06/RESULTADOS.md`). Con una sola velocidad no se pueden distinguir.

## 2. Qué averiguar del INTI antes de ir

| Pregunta | Para qué |
|---|---|
| Tipo de equipo y resolución | saber si llegamos a los escalones de 10 nm |
| Velocidad máxima y frecuencia de muestreo | límite de las pruebas 2, 4 y 6 |
| Entrada de disparo externo (TTL) o salida de reloj | sincronizar con el RP2040 (ver §3) |
| Exportación: CSV con tiempo de cada muestra, captura continua y su duración | procesar los datos |
| Compensación del aire (T, P, HR) y clima del laboratorio | 1 K son unos 0,3 µm en 300 mm |
| Retrorreflector o espejo, tamaño | diseñar el soporte en el carro |
| Latencia del instrumento | medir fase en el Bode |
| Incertidumbre declarada o certificado | límite de lo que se puede afirmar en el informe |

## 3. Sincronismo y adquisición

- A 2 mm/s, cada microsegundo de desfase entre los relojes equivale a 2 nm.
- La telemetría del RP2040 es de 1 kHz y el perfil corre a 2 kHz. El lazo servo es de unos 16 kHz.
- Si el INTI acepta disparo externo, se saca un pulso de un GPIO del RP2040 al inicio de cada secuencia.
- Si no lo acepta, los tiempos se alinean después por correlación entre v(t) de ambas señales. Sirve para las pruebas 1, 2 y 5, y no para la fase del Bode ni para fijar el instante exacto de una inversión.
- Cada serie arranca con un cero marcado. Si el interferómetro es incremental, un corte del haz pierde la cuenta, así que las series largas se dividen.
- Se anotan T, P y HR, y la temperatura del husillo y del granito (termistor pegado).

## 4. Pruebas

### Prueba 1. Mapa de error en rampa lenta

**Objetivo.** Medir e_m(x) a lo largo del recorrido.

**Secuencia.**
1. Cero del interferómetro en la posición inicial.
2. Rampa a 0,5–1 mm/s a lo largo de todo el recorrido útil, y vuelta.
3. Tres ida y vuelta completos.

A esa velocidad el error dinámico es mínimo, y e_m queda como error casi puramente geométrico. A 0,5 mm/s, 300 mm son 10 min por pasada.

**Análisis.**
1. Se remuestrea e_m(x) con paso fijo (por ejemplo 10 µm).
2. Se resta la tendencia lineal. Su pendiente es el error de escala en ppm, mezcla de paso del husillo y dilatación.
3. Se calcula la FFT en el espacio (ciclos/mm) y se buscan picos en 0,2 (husillo), 40 (paso de 25 µm), 12 (imanes) y 6 (encoder).
4. Se compara ida contra vuelta. La diferencia es pérdida de movimiento más retardo por fricción.

**Qué decide.**
- La tabla de compensación del husillo.
- Si la no linealidad del encoder de 4,4–5,2 µm RMS es real. Esa cifra salió de `encoder_sweep` contra el campo comandado y sobreestima (`docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md`).

**Requisitos.** No necesita sincronismo, porque se une por posición. Muestreo mínimo de 100 S/s. Duración 30–60 min.

### Prueba 2. Espectro de velocidad y ripple

**Objetivo.** Ver qué componentes tiene el movimiento real y de dónde vienen.

**Secuencia.** Velocidad constante a 1, 2 y 5 mm/s. Se analiza solo la ventana central, sin aceleración ni frenado.

**Análisis.**
- No se deriva x_i, porque la derivada amplifica el ruido. Se usa el residuo de posición, x_i − v·t − recta ajustada, y se calcula su densidad espectral.
- Se hace lo mismo con x_e y se comparan:

| Aparece en | Significa |
|---|---|
| solo x_e | artefacto del sensor |
| solo x_i | mecánico (husillo, guía) |
| ambos y se mueve con la velocidad | espacial (paso, imanes) |
| ambos y no se mueve | dinámica del lazo |

- Se compara la velocidad media real con la comandada. Eso da el error de escala en el tiempo.

**Qué decide.**
- Si la huella del paso del motor que el anti-cogging redujo (1,46 → 0,06 µm) también está removida en el carro.
- Si la oscilación de 205–214 Hz es del carro.
- Cuánto vale el ripple de velocidad real.

**Requisitos.** Muestreo de 1–5 kS/s como mínimo para llegar a 300 Hz. Sincronismo no crítico. Duración unos 15 min.

### Prueba 3. Inversión en movimiento

**Objetivo.** Medir la pérdida de movimiento dinámica.

**Secuencia.** Perfil trapezoidal que avanza, frena, invierte y retrocede. Velocidades de 0,5, 2 y 5 mm/s, 2 o 3 aceleraciones, en 3 posiciones del recorrido.

**Análisis.** Se grafica e_m contra la posición del encoder. Con la inversión se abre un lazo de histéresis. Se miden:
- el ancho del lazo (pérdida de movimiento dinámica);
- el tramo plano, donde el encoder avanza y el carro no;
- el pico de e_m y cuánto tarda en recuperarse;
- la diferencia con el backlash estático.

**Qué decide.** Si el juego dinámico es mayor que el estático, hay fricción o elasticidad que dependen de la velocidad. También muestra si el integrador del lazo (`POS_KI`) se pasa en la inversión.

**Requisitos.** Conviene sincronismo por hardware. Sin él, la inversión se localiza por el cambio de sentido de x_i y x_e, que ocurre en el mismo evento. Muestreo de 1 kS/s, 20 min.

### Prueba 4. Respuesta en frecuencia (Bode)

**Objetivo.** Medir la dinámica del carro real, con resonancias del husillo y del acople.

**Secuencia.** Se comanda x_c = x_0 + A·sen(2πft) con A = 1 µm y frecuencias de 1 a 200 Hz en escala logarítmica (1, 2, 5, 10, 20, 50, 100, 150, 200 Hz). Se repite con A = 100 nm para ver la linealidad. También sirve un chirp.

Con A = 1 µm a 100 Hz, la velocidad pico es 0,63 mm/s y la aceleración 0,39 m/s². El perfil corre a 2 kHz, así que el techo práctico es de 200–300 Hz.

**Análisis.** Ganancia y fase de x_e/x_c y de x_i/x_c. La diferencia entre ambas es la transmisión mecánica entre motor y carro. Un pico en x_i/x_e marca la resonancia de la rigidez axial del husillo con la masa del carro. El ancho de banda a −3 dB sale de x_i/x_c.

**Qué decide.** Permite volver a elegir `VEL_KP` y `POS_KP` con datos del carro. Hasta ahora se eligieron con el encoder, y el umbral de `VEL_KP` entre 0,12 y 0,16 puede cambiar.

**Requisitos.**
- La ganancia se mide sin sincronismo. La fase necesita sincronismo por hardware: a 100 Hz, 1° son 28 µs. También hay que conocer la latencia del instrumento.
- **Pendiente de resolver:** los comandos del firmware que vimos son `G0`, `G1`, `G28`, `G90`, `G91` y `M203`, `M55`–`M59`, y no hay un comando de senoide. Hay que decidir si se agrega uno al firmware o si se transmiten movimientos chicos desde la PC (con la latencia del USB como límite).
- Muestreo de 2–5 kS/s, 30 min.

### Prueba 5. Escalones y asentamiento

**Objetivo.** Ver cómo responde el carro a un escalón y si hay stick-slip.

**Secuencia.** Escalones de 10 nm, 50 nm, 100 nm, 1 µm, 10 µm, 100 µm y 1 mm, en ambos sentidos, con 5–10 s de espera después de cada uno. Se puede partir de `tools/escalera_nm.py`.

**Qué se mide.**
- Tiempo de subida, sobrepaso y tiempo de asentamiento a ±10 nm.
- Error final contra el objetivo a cada escala.
- Stick-slip: el carro no se mueve y después salta. En octubre vimos saltos de 40–80 nm con unos 2 s de carga y 8 s de amortiguación (`docs/CAMBIOS_2026-10-23_PRECISION.md`), con ganancias viejas. No se verificó con las ganancias finales.
- Si los pasos de 10 nm que midió el encoder (10,02 ± 0,41 nm, N = 55) también los da el carro.

**Cuidado con la resolución.** Un paso de 10 nm son 10 cuentas si el instrumento resuelve 1 nm, y el ruido del laboratorio puede ser de varios nm. Se promedia la espera (por ejemplo 1 s a 1 kHz) y el piso de ruido del instrumento se mide primero, con el motor apagado.

**Requisitos.** Es la que menos pide: las escalas de tiempo son de segundos y alcanza con alineación manual. Muestreo de 1 kS/s, 30 min.

### Prueba 6. Aceleración y rigidez axial

**Objetivo.** Medir cuánto se flexiona el husillo al acelerar.

**Secuencia.** Perfiles trapezoidales a 2 y 5 mm/s con distintas aceleraciones.

**Análisis.** Durante la aceleración, e_m ≈ (m·a + F_fricción)/k. Graficando e_m contra a se obtiene la rigidez axial k, y la fuerza de fricción en la ordenada.

**Qué decide.** A 3–5 mm/s el torque se saturó (salida > 70° entre el 1 y el 40 % del tiempo, según la velocidad; `*_acoplado_v*_resumen.txt` del 03/11). Esta prueba dice si en esos tramos el carro pierde posición de verdad.

**Requisitos.** Conviene sincronismo. Muestreo de 1 kS/s, 15 min.

### Prueba 7. Calentamiento en movimiento

**Objetivo.** Medir cuánto se dilata el husillo con el uso.

**Secuencia.** Ida y vuelta continuo durante 30 min, registrando e_m al cruzar los mismos puntos. Con termistor en el husillo y en el granito.

**Qué decide.** Cuánto cambia la escala de e_m con el tiempo. Un encoder del lado del motor no ve la dilatación del husillo. Si el INTI no puede dedicar tanto tiempo, queda pendiente para hacer en el laboratorio con otro método.

**Requisitos.** Muestreo de 10 S/s, 30 min o más.

## 5. Resumen

| Prueba | Sincronismo por hardware | Muestreo mínimo | Duración |
|---|---|---|---|
| 1. Mapa en rampa | No | 100 S/s | 30–60 min |
| 2. Espectro de velocidad | No | 1–5 kS/s | 15 min |
| 3. Inversión | Conveniente | 1 kS/s | 20 min |
| 4. Bode | Para la fase | 2–5 kS/s | 30 min |
| 5. Escalones | No | 1 kS/s | 30 min |
| 6. Aceleración | Conveniente | 1 kS/s | 15 min |
| 7. Calentamiento | No | 10 S/s | 30+ min |

## 6. Orden para una sola visita

1. Piso de ruido del instrumento: motor apagado y unos minutos de registro. Si el entorno mete más de unos pocos nm, no se pueden afirmar los escalones de 10 nm.
2. Prueba 1 (mapa de error).
3. Pruebas estáticas (ISO 230-2 y backlash), en el otro documento.
4. Prueba 5 (escalones).
5. Pruebas 2, 3 y 6, si hay buena sincronía.
6. Prueba 4 (Bode), solo con sincronismo por hardware y un modo de comandar la senoide.
7. Prueba 7, si hay horas disponibles.

La compensación del husillo se calcula en el laboratorio a partir del mapa de la prueba 1. Si funciona, conviene una segunda visita corta para medir el después.

## 7. Por hacer antes de ir

- Scripts de secuencias en `tools/` para las pruebas 1, 2, 3 y 5, con la lista de posiciones y los tiempos medidos en un ensayo solo con el encoder.
- Soporte del reflector en el carro, a la altura del punto de trabajo. Se anota el desplazamiento de Abbe que quede.
- GPIO de sincronismo en el RP2040, si el INTI acepta disparo externo.
- Resolver cómo comandar la senoide para la prueba 4.
