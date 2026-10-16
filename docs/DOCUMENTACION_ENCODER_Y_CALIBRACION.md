# Caracterización del encoder y calibración del eje: guía explicativa

> **Qué es este documento.** Explica, sin suponer conocimiento previo del proyecto, cómo se mide y se corrige la no linealidad del encoder magnético del eje lineal de micromaquinado. Cubre las herramientas `encoder_sweep.py`, `plot_periods.py` y `capture_calibration.py`, el barrido de caracterización y el proceso de calibración (`M56`) que construye las tablas que usa el lazo cerrado.
>
> **Estado del repositorio que se documenta:** sesión del 29/09/2026. En la placa está cargado el firmware de barrido (`src/main.cpp` temporal). El firmware real está respaldado en `main_firmware_completo.cpp.bak`.

---

## 0. Cómo leer este documento

El documento va de lo general a lo particular. Si no conocés el proyecto, leé las secciones 1 y 2 completas. Si solo querés saber qué significa una columna del CSV, andá directo a la sección 4.7. Si te interesa cómo funciona la calibración, andá a la sección 6.

Cada dato importante lleva una marca que indica de dónde sale:

| Marca | Significa |
|---|---|
| **[código]** | Lo leí directamente en el firmware o en los scripts. |
| **[medido]** | Lo calculé sobre el archivo `mediciones/encoder_sweep_20260929_182151.csv`. |
| **[notas]** | Viene de `SESION_2026-09-29.md` o de `HISTORIAL_DE_CAMBIOS_FIRMWARE.md`. No lo verifiqué. |
| **[estimación]** | Cálculo propio a partir de lo anterior. Sirve como orden de magnitud, no como dato firme. |

Contenido:

1. Contexto del proyecto y glosario
2. La idea central: por qué se caracteriza y se calibra
3. Mapa de herramientas
4. El barrido de caracterización (`encoder_sweep`)
5. Análisis por sectores (`plot_periods`)
6. La calibración (`M56` y `capture_calibration`)
7. Decisiones de diseño y resolución alcanzable
8. Puntos de atención y recomendaciones
9. Apéndices (archivos, conversiones, flujo de trabajo)

---

## 1. Contexto del proyecto y glosario

### 1.1 El proyecto en breve

El proyecto diseña, construye y caracteriza metrológicamente un **eje de posicionamiento lineal de precisión micrométrica** para micromaquinado. Su desempeño se compara en **lazo abierto** y en **lazo cerrado** contra un interferómetro de diodo construido por los alumnos. El resultado se valida de forma indirecta maquinando una estructura de prueba y midiéndola al microscopio. El eje está pensado para poder ampliarse más adelante a una configuración XY.

### 1.2 El eje físico

| Elemento | Dato |
|---|---|
| Transmisión | Husillo de bolas, **2 mm de avance por vuelta** |
| Recorrido útil | ~260 mm (el firmware limita los comandos a 0–255 mm) **[notas][código]** |
| Motor | Paso a paso de 1,8° por paso (200 pasos por vuelta), **50 pares de polos** **[código]** |
| Driver del motor | TB6612, fuente de 10 V / 1 A **[notas]** |
| Sensor de posición | Encoder magnético MT6835, 21 bits, comunicación SPI |
| Rueda de imanes | **60 imanes** cilíndricos de 3 × 5 mm, alternados (norte-sur), pegados al eje del motor. Paso entre imanes 3 mm, radio del anillo 28,9 mm **[código][notas]** |
| Microcontrolador | RP2040 (entorno `pico` de PlatformIO), programado con el framework Arduino-Pico **[código]** |
| Finales de carrera | No hay. El homing detecta el tope mecánico porque el encoder deja de seguir al motor. |

**Cómo se mide la posición.** El motor gira el husillo y el carro avanza. En el eje del motor hay pegada una rueda con 60 imanes alternados. Frente a ella, a poca distancia, está el chip MT6835, que mide la dirección del campo magnético que le llega. Cuando la rueda gira, esa dirección da una vuelta completa cada dos imanes. Por eso un giro del motor produce **30 vueltas del ángulo del sensor**, y no una sola. Eso multiplica la resolución, pero introduce un problema de linealidad que se explica en la sección 2.

**Importante:** el encoder mide el giro del **motor**, no la posición del **carro**. La posición del carro se deduce del giro del motor suponiendo que el husillo avanza exactamente 2 mm por vuelta. Los errores del husillo (paso, juego) no los ve el encoder.

### 1.3 Glosario

| Término | Qué significa |
|---|---|
| **Lazo abierto** | El motor recibe una orden y se asume que la cumple. No se usa el sensor para corregir. |
| **Lazo cerrado** | El sensor mide la posición real y un controlador (PID) corrige la orden en cada instante. |
| **Campo (ángulo de campo)** | El driver escribe corrientes con forma de seno y coseno en las dos bobinas del motor. La suma es un campo magnético que apunta en una dirección. El rotor tiende a alinearse con esa dirección. El ángulo de esa dirección es el "ángulo de campo". |
| **Ángulo eléctrico vs. mecánico** | Con 50 pares de polos, por cada vuelta mecánica del motor el campo da 50 vueltas eléctricas. 1 rad mecánico equivale a 50 rad eléctricos. |
| **`field_rad`** | Ángulo eléctrico del campo que el firmware **ordena** (no es una medición). |
| **`motor_pos`** | Posición del motor en radianes mecánicos. En la calibración se define como `(campo − campo_inicial) / 50`. |
| **Atraso del rotor** | El rotor no sigue al campo exactamente: por la fricción queda un poco detrás en el sentido de giro. Es de ~0,45° mecánicos y cambia de signo al invertir el giro. |
| **Encoder MT6835** | Sensor magnético absoluto. Entrega un número de 21 bits (0 a 2 097 151) que representa el ángulo del campo dentro de **un período**. |
| **Período (del encoder)** | Un ciclo completo del ángulo del sensor: 360° del sensor. Corresponde a **2 imanes** (6 mm sobre la rueda). |
| **Cuenta** | La unidad mínima del encoder: 1/2²¹ de período. |
| **Desenrollado (unwrap)** | El chip solo informa el ángulo dentro de un período. Para saber cuántos períodos pasaron, el firmware suma las diferencias entre lecturas sucesivas y lleva un contador absoluto de cuentas. |
| **Campo débil** | Bit de estado que el chip activa cuando el campo magnético que recibe es demasiado débil. El chip no informa cuánto vale el campo, solo ese aviso. |
| **Overspeed** | Bit de estado que el chip activa cuando el imán gira más rápido de lo que puede convertir. |
| **CRC** | Código de verificación que acompaña cada lectura. Si no coincide, la lectura llegó dañada. |
| **LUT (tabla de búsqueda)** | Tabla de números que el firmware consulta para convertir un valor en otro, interpolando entre puntos. |
| **Homing** | Procedimiento que lleva el carro contra el tope mecánico para definir la posición cero. |
| **Teleplot** | Programa para ver señales en vivo. Los scripts le reenvían datos por UDP. |

### 1.4 Equivalencias de escala (para no perderse entre unidades)

Todas estas cantidades describen el mismo movimiento. Conviene tenerlas a mano:

| Magnitud | Equivale a |
|---|---|
| **1 vuelta de motor** | 360° mecánicos = **2 mm de carro** = 30 períodos del encoder = 60 imanes = 50 ciclos eléctricos = 200 pasos completos |
| **1 período del encoder** (360° del sensor) | 12° de motor = **66,7 µm de carro** = 2 imanes = 6 mm sobre la rueda |
| **1° del sensor** | 0,0333° de motor = **0,185 µm de carro** |
| **1 ciclo eléctrico** | 7,2° de motor = 40 µm de carro |
| **1 paso completo del motor** | 1,8° de motor = 10 µm de carro |
| **1 cuenta del encoder** | 0,000172° del sensor = **0,032 nm de carro** |

Una advertencia sobre una de estas unidades: el CSV de `encoder_sweep` tiene una columna `x_mm`. **No es la posición del carro.** Es el arco recorrido sobre la rueda de imanes (180 mm por vuelta de motor). Se explica en la sección 4.7.

---

## 2. La idea central: por qué se caracteriza y se calibra

### 2.1 El problema

Si el sensor fuera perfecto, su ángulo crecería en proporción exacta al giro del motor: un motor que gira 12° haría avanzar el sensor exactamente 360° (un período). En la práctica no pasa. El ángulo medido tiene una **ondulación repetible**: dentro de cada período va un poco adelantado o atrasado respecto de la recta ideal, y además varía de un imán al otro. En la mejor medición tomada el desvío llega a ±10–30° del sensor **[medido]**.

Si el firmware convirtiera el ángulo del sensor en posición con una simple proporción, heredaría ese error. El error es **repetible**, es decir, es siempre el mismo en la misma posición, y eso permite corregirlo con una tabla.

### 2.2 La solución en tres pasos

```
 PASO 1: CARACTERIZAR                PASO 2: CALIBRAR               PASO 3: USAR
 (encoder_sweep, plot_periods)       (M56, capture_calibration)     (lazo cerrado: M17, G1)

 ¿El sensor está sano?               Medir la curva real            Cada ciclo (~22 kHz):
 - sin campo débil                   "lectura del encoder →         lectura del encoder
 - sin saltos                           posición del motor"             │
 - 30 períodos por vuelta            y guardarla como tabla             ▼
 - error pequeño y repetible                                        tabla → posición
                                                                        │
 Si no está sano, no tiene                                              ▼
 sentido calibrar.                                                  PID → ángulo de campo
```

- **Caracterizar** (sección 4 y 5) es un diagnóstico. Responde si el sensor y la rueda de imanes están en condiciones: sin avisos del chip, sin saltos, con el número correcto de períodos. Se hace con un firmware aparte.
- **Calibrar** (sección 6) es la medición que produce las tablas. Se hace con el firmware real y queda guardada en la placa.
- **Usar** la tabla ocurre en el lazo cerrado, en cada ciclo de control.

### 2.3 Una limitación de fondo

En ambos pasos, la "posición verdadera" con la que se compara el encoder **es el ángulo de campo que el propio firmware ordena**. No es una medición independiente. Esto implica que:

- El atraso del rotor y las irregularidades del accionamiento (micropasos) quedan mezclados con el error del encoder.
- Los errores del husillo no aparecen en ningún lado.
- Para una caracterización metrológica real hace falta una referencia externa. En este proyecto es el **interferómetro**.

Los resultados de este documento deben leerse como la calidad del sensor **relativa al campo comandado**, no como exactitud absoluta del eje.

---

## 3. Mapa de herramientas

### 3.1 Resumen

| Herramienta | Dónde corre | Firmware que necesita en la placa | Qué hace | Qué produce |
|---|---|---|---|---|
| `tools/encoder_sweep.py` | PC | **De barrido** (`src/main.cpp` temporal) | Manda `G56`, captura un barrido en lazo abierto y lo analiza | CSV y PNG en `mediciones/` |
| `tools/plot_periods.py` | PC (sin puerto serie) | Ninguno | Re-analiza un CSV de `encoder_sweep` por sectores de la rueda | PNG `*_periodos_N.png` y una tabla en consola |
| `tools/capture_calibration.py` | PC | **Real** (`main_firmware_completo.cpp.bak`) | Manda `M56 P`, captura los datos de la calibración y los analiza | `calib.csv`, `calib_log.txt`, `calib.png` |
| `tools/send_gcode.py` | PC | Real | Manda comandos G-code en secuencia (herramienta auxiliar) | Respuestas en consola |

### 3.2 Dos firmwares distintos

La placa solo puede tener un programa a la vez:

- **Firmware de barrido** (hoy en `src/main.cpp`): hace una sola cosa. Reconoce únicamente el comando `G56`, que dispara el barrido de caracterización. **[código]**
- **Firmware real** (en `main_firmware_completo.cpp.bak`): el control completo del eje. Reconoce `G0/G1` (movimiento), `G28` (homing), `M56` (calibración), `M17/M18` (activar/desactivar motor), `M114/M119` (estado), `M120/M121` (telemetría y traza), entre otros. **[código]**

**Cuidado con los nombres:** `G56` (barrido de caracterización, solo en el firmware de barrido) y `M56` (calibración, solo en el firmware real) se parecen pero son comandos de distinto firmware. Si se manda `G56` al firmware real, responde `error: unknown command 'G56'`.

Para volver al firmware real **[notas]**:

```bash
cp main_firmware_completo.cpp.bak src/main.cpp
pio run -t upload
```

### 3.3 Requisitos comunes de uso

- El **puerto serie debe estar libre**. Hay que cerrar el monitor de PlatformIO y cualquier programa que lo tenga abierto. En Linux, un solo proceso puede abrir el puerto.
- Teleplot puede quedar abierto pero **sin conectarse al puerto serie**. Los scripts le reenvían los datos por UDP (puerto 47269).
- La velocidad del puerto es 921 600 baudios.
- Los scripts necesitan Python con `numpy`, `matplotlib` y `pyserial`.

---

## 4. El barrido de caracterización (`encoder_sweep`)

### 4.1 En una frase

Se hace girar el motor en lazo abierto, despacio y con un movimiento perfectamente definido, y se anota cada milisegundo qué ángulo informa el encoder. Después se compara lo medido con lo que mediría un encoder ideal.

### 4.2 Preparación

1. Cargar el firmware de barrido en la placa.
2. Cerrar cualquier programa que use el puerto serie.
3. **Marcar con fibrón el imán que está frente al sensor** antes de empezar. `plot_periods` numera los imanes desde esa posición inicial, y la marca permite ubicar físicamente cualquier problema que aparezca.
4. Ejecutar:

```bash
python3 tools/encoder_sweep.py                      # barrido nuevo
python3 tools/encoder_sweep.py --from-csv archivo   # re-graficar un CSV guardado
```

Opciones útiles: `--port` (por defecto `/dev/ttyACM0`), `--magnets` (60), `--pitch` (3 mm), `--pole-pairs` (50), `--no-teleplot`, `--teleplot-decimation N`. **Los valores de `--magnets`, `--pitch` y `--pole-pairs` tienen que coincidir con el montaje real**, porque determinan varias columnas calculadas (ver sección 8).

### 4.3 Qué hace la placa, paso a paso

Al arrancar, el firmware de barrido **[código]**:

1. Configura el SPI a 8 MHz, modo 3.
2. Inicializa el encoder con ancho de banda interno `BW = 5` e histéresis 4, y activa la verificación de CRC.
3. Inicializa el driver del motor con amplitud 0.

Al recibir `G56` **[código]**:

1. Imprime una línea `META,...` con los parámetros (pares de polos, cuentas por vuelta, vueltas, velocidad, amplitud) y la palabra `BEGIN`.
2. Sube la corriente del motor a 0,6 de forma gradual (200 ms) y espera 300 ms a que el rotor se asiente.
3. Pone a cero el contador de errores de CRC.
4. **Tramo de ida:** gira el campo 2 vueltas mecánicas en sentido positivo.
5. Espera 200 ms.
6. **Tramo de vuelta:** gira el campo 2 vueltas mecánicas en sentido negativo, volviendo al punto de partida.
7. Imprime `END,crc_errors=N`, baja la corriente y termina.

Durante cada tramo, el campo se actualiza cada 100 µs y se toma **una muestra cada 1 ms** (cada 10 actualizaciones).

### 4.4 El criterio de `field_rad`: cómo se genera el movimiento comandado

El ángulo de campo no se mide: se **genera** con un perfil trapezoidal de velocidad **[código]**:

| Parámetro | Valor | Equivale a |
|---|---|---|
| Velocidad de crucero | 0,25 vueltas/s de motor | 78,5 rad/s eléctricos, 12,5 ciclos eléctricos/s, 50 pasos completos/s, 0,5 mm/s de carro |
| Rampa de aceleración y de frenado | 0,5 s | Aceleración ≈ 157 rad/s² eléctricos |
| Recorrido por tramo | 2 vueltas de motor | 628,3 rad eléctricos, 4 mm de carro |
| Actualización del campo | cada 100 µs | 10 kHz |
| Amplitud (corriente) | 0,6 del máximo | |

En cada actualización, el firmware hace:

```
field = field + dir × v × dt        (dt = 100 µs nominales, dir = +1 o −1)
```

La velocidad `v` sube con la aceleración hasta el crucero. El frenado empieza cuando la distancia que falta es igual a `v²/(2·a)`, y la velocidad cae hasta un piso del 5 % de la máxima para terminar justo en la distancia pedida. Cada tramo dura ~8,5 s.

**Por qué el campo sirve de referencia.** Un motor paso a paso es sincrónico: el rotor sigue al campo con un pequeño atraso. Si el movimiento del campo es conocido y suave, se puede usar como referencia de posición sin depender del encoder. Esa independencia es lo que permite juzgar al encoder. Pero esta referencia tiene las limitaciones de la sección 2.3.

**Dos detalles numéricos [código][medido]:**

- `dt` es el valor nominal de 100 µs, no un tiempo medido. Lo que se imprime como tiempo (`t_s`) sí es el tiempo real de la placa.
- El campo se acumula en un número de punto flotante de 32 bits. Cerca de 500 rad eléctricos el redondeo cambia el avance de cada paso en una fracción pequeña. En el CSV, la velocidad real del campo en crucero se desvía entre −0,14 % y +0,06 % de la nominal según el rango del ángulo. **No afecta la medición**, porque las columnas derivadas se calculan con el valor de `field_rad` que realmente se aplicó.

### 4.5 Cómo se lee el encoder

En cada muestra, la placa hace una lectura de ángulo en ráfaga **[código]**:

- Trama SPI de 6 bytes (48 bits, unos 6 µs a 8 MHz).
- Bytes 0–1: comando de lectura de ángulo (registro 0x003).
- Bytes 2–4: **21 bits de ángulo** y **3 bits de estado**.
- Byte 5: **CRC-8** (polinomio 0x07).

El firmware recalcula el CRC con el ángulo y el estado recibidos. Si no coincide, activa un cuarto bit de estado (error de CRC) y suma uno a un contador.

**Qué valor se informa.** El barrido imprime el ángulo de 21 bits **tal cual lo entrega el chip** (de 0 a 2 097 151), sin desenrollar ni filtrar. Es la señal "cruda". Es distinto de la calibración, que usa cuentas absolutas desenrolladas (ver sección 6.4).

**Qué significan los bits de estado:**

| Bit | Nombre | Quién lo genera | Significado |
|---|---|---|---|
| 0 | `overspeed` | El chip | El campo gira más rápido de lo que el chip puede convertir |
| 1 | `weak` (campo débil) | El chip, con un umbral interno fijo | El campo magnético recibido es débil. La lectura se entrega igual, marcada como poco confiable |
| 2 | `undervolt` | El chip | Subtensión de alimentación |
| 3 | `crc_err` | El microcontrolador | El CRC de la trama no coincide |

El chip **no informa la intensidad del campo en mT**: solo el bit de campo débil.

### 4.6 Qué hace el script de la PC

Cada línea que llega de la placa tiene la forma:

```
D,<t_us>,<sentido>,<campo [rad]>,<ángulo crudo del chip>,<estado>
```

El script `encoder_sweep.py` hace tres cosas **[código]**:

1. **Captura** las líneas `D,...` por el puerto serie hasta recibir `END`, y reenvía cada quinta muestra a Teleplot.
2. **Construye la tabla** (función `build_table`). Calcula las columnas que no vienen de la placa. El paso clave es construir el encoder ideal:
   - Primero decide si el ángulo del sensor crece o decrece junto con el campo (prueba los dos signos y se queda con el que muestra más coherencia con los datos).
   - Después calcula, **por separado para cada sentido**, un offset constante (promedio circular de la diferencia entre lo medido y el ángulo esperado). Así el atraso del rotor queda absorbido y el error resultante no tiene sesgo en ninguno de los dos sentidos.
3. **Analiza y grafica** (función `analyze`): imprime un resumen y genera un PNG de 8 paneles.

### 4.7 Las columnas del CSV, una por una

El archivo `mediciones/encoder_sweep_AAAAMMDD_HHMMSS.csv` tiene una fila por muestra (unas 17 000 por corrida) y estas 13 columnas:

| Columna | Unidad | Origen | Cómo se obtiene | Qué representa |
|---|---|---|---|---|
| `t_s` | s | **Medido** (reloj de la placa) | `time_us_64()` de la placa menos el instante de inicio, tomado justo después de la lectura SPI. El instante cero es común a los dos tramos. | Tiempo desde el inicio del barrido. El paso es de 1,000 ms (percentil 1–99: 0,995–1,005 ms). El único hueco es la pausa de 200 ms entre tramos. **[medido]** |
| `dir` | ±1 | Firmware | +1 en el tramo de ida, −1 en el de vuelta | Sentido de giro |
| `field_rad` | rad eléctricos | **Comandado** (no medido) | El ángulo de campo generado por el perfil de la sección 4.4 | Referencia de posición. Arranca donde quedó la corrida anterior (−0,0113 rad en la corrida de ejemplo) y llega a 628,31 rad en la ida. |
| `motor_deg` | grados mecánicos | **Calculado** | `(field_rad − field_rad[0]) / 50`, pasado a grados | Ángulo de motor comandado, relativo a la primera muestra. Va de 0° a 720° y vuelve a 0°. |
| `x_mm` | mm | **Calculado** | `motor_deg/360 × (60 imanes × 3 mm)` | **Arco sobre la rueda de imanes** (180 mm por vuelta de motor), de 0 a 360 mm. **No es el recorrido del carro.** El carro avanza `motor_deg/360 × 2 mm`, o sea 4 mm por tramo. |
| `raw` | cuentas | **Medido** | Ángulo de 21 bits del chip, sin procesar | Lectura directa del encoder. Con `status`, es lo único medido. |
| `enc_deg` | grados del sensor | **Calculado** | `raw / 2²¹ × 360` | La misma lectura expresada en grados de período (0–360°). |
| `ideal_deg` | grados del sensor | **Calculado (modelo)** | Fase del encoder ideal: `motor_deg × 30` con un offset constante por sentido, reducido a 0–360° | Lo que mediría un encoder perfectamente lineal anclado a esta corrida. **Es un ajuste, no una medición.** |
| `error_deg` | grados del sensor | **Calculado** | `enc_deg − ideal_deg`, llevado a ±180° | Desvío del encoder respecto de la recta ideal. Se expresa en grados de período (ver conversiones abajo). |
| `overspeed` | 0/1 | **Medido** (chip) | Bit 0 del estado | Aviso de giro demasiado rápido |
| `weak` | 0/1 | **Medido** (chip) | Bit 1 del estado | Aviso de campo débil |
| `undervolt` | 0/1 | **Medido** (chip) | Bit 2 del estado | Aviso de subtensión |
| `crc_err` | 0/1 | **Medido** (MCU) | Bit 3, resultado de la verificación de CRC | Aviso de lectura dañada |

**Cómo convertir `error_deg` a unidades útiles.** Como 1° del sensor equivale a 0,0333° de motor y a 0,185 µm de carro:

```
error en grados de motor = error_deg / 30
error en µm de carro     = error_deg × 0,185
```

**Cómo interpretar `error_deg` con cuidado.** Este error no es solo "defecto del encoder". Incluye también:

- Variaciones del atraso del rotor dentro de cada sentido.
- Irregularidades del micropaso del motor.
- Diferencias entre imanes y excentricidad de la rueda.
- Cualquier desvío del campo real respecto de la orden.

Por eso **sobreestima** el error propio del encoder. Y como el offset se ajusta sobre la misma corrida, el error promedio de cada sentido es cero por construcción.

### 4.8 Qué mira el resumen de consola

Al terminar, el script imprime **[código]**:

- Cantidad de muestras.
- Cantidad y porcentaje de muestras con campo débil, y cantidad de overspeed, subtensión y errores de CRC.
- **Saltos:** cuántas veces el ángulo medido entre dos muestras consecutivas difiere en más de 45° de lo esperado.
- Error RMS y máximo contra el ideal, por separado para ida y vuelta, en grados del sensor y en grados de eje.
- **Períodos por vuelta:** la pendiente del ángulo desenrollado del sensor contra el ángulo de motor. Tiene que dar un entero cercano a 30. Si no, el sensor está perdiendo períodos.

### 4.9 Los 8 gráficos del PNG

| Panel | Qué muestra | Para qué sirve |
|---|---|---|
| 1 (arriba izq.) | Ángulo medido contra posición a lo largo de la rueda, en una ventana de 12 mm (2 períodos) de la ida, con la recta ideal | Ver la forma de la ondulación. Equivale al gráfico de la simulación |
| 2 | Ángulo crudo en todo el barrido, ida y vuelta | Ver si se pierden períodos y dónde aparece el campo débil |
| 3 | Error contra posición comandada | Ver la no linealidad a lo largo de toda la corrida |
| 4 | Error contra fase dentro del período | Ver la forma de la no linealidad dentro de un período |
| 5 | Porcentaje de campo débil y overspeed según la fase del período | Ver si el aviso aparece siempre en la misma fase |
| 6 | Campo débil a lo largo de la vuelta (1 barra por imán) | Localizar imanes problemáticos |
| 7 | Salto de lectura entre muestras consecutivas | Detectar saltos de 180° |
| 8 | Bits de estado en el tiempo | Ver cuándo aparece cada aviso |

### 4.10 Resultado de ejemplo: corrida de las 18:21

Archivo `encoder_sweep_20260929_182151.csv`. Es la mejor corrida registrada.

| Dato | Valor | Origen |
|---|---|---|
| Muestras | 16 976 (8488 por tramo) | [medido] |
| Duración de cada tramo | ~8,5 s | [medido] |
| Campo débil, overspeed, subtensión, errores de CRC | 0 en las cuatro | [medido] |
| Períodos de encoder por vuelta de motor | 29,98 (se espera 30) | [medido] |
| Error RMS, ida | 10,4° del sensor = 0,35° de motor ≈ **1,9 µm** de carro | [medido] |
| Error RMS, vuelta | 11,2° del sensor = 0,37° de motor ≈ **2,1 µm** de carro | [medido] |
| Error máximo | 33° del sensor ≈ 6,1 µm de carro | [medido] |
| Diferencia entre extremos del error | 62,8° del sensor ≈ 11,6 µm de carro | [medido] |
| Avance del motor entre muestras | 0,09° de motor ≈ 0,5 µm de carro | [medido] |

Cada período del encoder queda cubierto por unas 133 muestras. La corrida muestra una ondulación suave y repetible, sin saltos ni avisos. Esa es la condición buscada antes de calibrar.

**Qué falta confirmar.** Las notas dicen que falta confirmar cómo estaba sostenido el sensor en esta corrida. La de las 18:13 se hizo con el sensor sostenido a mano **[notas]**.

### 4.11 Historia: cómo se llegó a una medición sana

Las primeras corridas del 29/09 fueron malas por la orientación del encoder, y sirven de ejemplo de qué se ve cuando algo está mal **[notas]**:

| Corrida | Campo débil | Overspeed | Saltos | Error RMS (sensor / eje) | Períodos por vuelta | Nota |
|---|---|---|---|---|---|---|
| 17:36 | 10,4 % | 701 | 317 | 65° / 2,2° | 22 (inválido) | Primer barrido tras el cambio de orientación |
| 18:02 | 17,7 % | 1990 | 472 | 103° / 3,4° | 20 (inválido) | |
| 18:04 | 18,2 % | 2092 | 528 | 100° / 3,4° | 15 (inválido) | |
| 18:13 | 0 % | 0 | 0 | 15° / 0,5° | 30,04 | Sensor sostenido a mano |
| 18:21 | 0 % | 0 | 0 | 10,4° / 0,35° | 29,98 | La mejor |

La causa de las corridas malas era que el encoder estaba girado 90° respecto del diseño original, por lo que medía el campo en un plano equivocado y en una zona donde es débil y deformado. Con el sensor reubicado, la medición quedó limpia. Una lectura útil de la tabla: **si los períodos por vuelta no dan un entero cercano a 30, o hay muchos saltos, el problema es físico (montaje o distancia), y ningún filtro de firmware lo arregla.**

---

## 5. Análisis por sectores (`plot_periods`)

### 5.1 Qué hace

`plot_periods.py` toma un CSV de `encoder_sweep` y lo reorganiza **por sectores de la rueda**, para responder dos preguntas: ¿dónde está el problema? y ¿es repetible? No usa el puerto serie ni la placa.

```bash
python3 tools/plot_periods.py mediciones/encoder_sweep_XXXX.csv
python3 tools/plot_periods.py archivo.csv --periods-per-window 1   # ventanas de 2 imanes
```

### 5.2 Cómo funciona

**[código]**

1. Calcula la posición dentro de la vuelta (`motor_deg` módulo 360°, pasada a mm sobre una rueda de 180 mm) y el número de vuelta (primera o segunda).
2. Descarta el primer y el último segundo de cada sentido, que corresponden a las rampas de arranque y frenado.
3. Divide la vuelta en **ventanas** de 2 períodos (12 mm, 4 imanes). Con 60 imanes salen 15 ventanas.
4. Dibuja una fila por ventana. La columna izquierda es la ida y la derecha es la vuelta.
5. En cada ventana superpone la **vuelta 1** (círculos grandes y claros) y la **vuelta 2** (puntos chicos), más la línea ideal en naranja. Marca con cruces rojas las muestras con campo débil y con signos "+" magenta las de overspeed.

El título de cada panel resume la ventana: qué imanes cubre, qué ángulo de la vuelta del motor ocupa, el porcentaje de campo débil, el error RMS y las "muestras corridas" (muestras con error de 120° o más, es decir, lecturas que cayeron cerca de medio período fuera de lugar). El RMS se calcula solo con las muestras de error menor a 120°.

Además imprime una tabla en consola, una fila por ventana y sentido, con esos mismos números. El PNG se guarda junto al CSV como `*_periodos_2.png`.

### 5.3 Cómo leerlo

- **Si las dos vueltas coinciden en un panel**, la deformación es propia de ese sector: viene de los imanes o del montaje. Es **repetible**, y por lo tanto la calibración la puede corregir.
- **Si no coinciden**, hay ruido, patinaje o un efecto que no se repite, y la calibración no lo corrige.
- Los imanes se numeran desde el que estaba frente al sensor al comenzar el barrido (imán 0). Por eso hay que marcarlo antes de cada corrida.

En la mejor corrida, el error por ventana de 4 imanes iba de ~6° del sensor (imanes 4–7, 28–31 y 56–59) a 12–18° (imanes 12–23). Es una modulación suave de una vez por vuelta, compatible con excentricidad residual de la rueda o con diferencias entre imanes. Es repetible entre vueltas y entre ida y vuelta **[notas]**.

---

## 6. La calibración (`M56` y `capture_calibration`)

### 6.1 Para qué sirve

La calibración mide, con el firmware real, la curva "lectura del encoder → posición del motor" y la guarda como una tabla. Con esa tabla el lazo cerrado convierte cada lectura del sensor en una posición corregida. También construye una segunda tabla, "posición → ángulo de campo", que el lazo cerrado necesita para saber hacia dónde apuntar el campo.

### 6.2 El comando

```
M56 [S] [P] [D<±1>]
```

| Parámetro | Significado |
|---|---|
| `S` | Guardar las tablas en la memoria flash de la placa (sobrevive a un reinicio) |
| `P` | Imprimir cada muestra por serie (es lo que usa `capture_calibration.py`) |
| `D` | Sentido de búsqueda del tope en el homing. Por defecto −1. `D1` todavía no funciona bien **[notas]** |

`M56` incluye el homing. Si el carro arranca en el extremo opuesto al tope, el homing solo puede tardar hasta unos 4,5 minutos, y la medición propiamente dicha ~45 s **[notas]**. Cuando termina bien responde `ok`; si falla, `error: calibracion fallida`. Mientras el eje no esté calibrado, el firmware no deja activar el lazo cerrado (`M17` responde `error: eje no calibrado, correr M56 primero`) **[código]**.

### 6.3 Qué hace la placa durante `M56`

**Paso 1: homing [código][notas].** El motor empuja contra el tope mecánico a 3 rad/s de motor (~0,95 mm/s) con corriente reducida (0,4). El firmware detecta que llegó al tope porque el encoder deja de avanzar aunque el campo siga girando. Si el encoder no sigue al motor desde el principio (por ejemplo, una rueda floja), el homing aborta. Al detectar el tope, el firmware pone a cero el contador de períodos del encoder (conserva solo el resto dentro del período), que sirve como referencia repetible de la posición home, y retrocede lo que avanzó el campo mientras se confirmaba el bloqueo.

**Paso 2: barrido de ida [código].** Desde el home, el campo gira **+370° de motor** a 20 rad/s eléctricos (0,4 rad/s de motor, 0,064 vueltas/s, 0,127 mm/s de carro). Dura ~16 s. El movimiento es continuo: no se frena entre muestras.

**Paso 3: barrido de vuelta [código].** Gira −370° a la misma velocidad.

**Paso 4: regreso [código].** Vuelve al punto inicial al triple de velocidad, sin exagerar para que el rotor no patine y el encoder pierda el seguimiento.

Se barren 370° y no 360° porque la tabla tiene que cubrir **al menos una vuelta completa** (30 períodos): el código aborta si el rango medido es menor. Los 10° extra son un margen **[estimación]**.

Los parámetros están en `src/hw_config.h`:

| Parámetro | Valor | Significado |
|---|---|---|
| `CALIBRATION_RANGE` | 370° | Recorrido del barrido, en grados de motor |
| `CALIBRATION_FIELD_VELOCITY` | 20 rad/s | Velocidad del campo durante la calibración |
| `ENCODER_LUT_SIZE` | 1024 | Cantidad de puntos de la tabla principal |

### 6.4 Qué se mide en cada muestra

Durante los barridos, el campo avanza con el reloj (`campo = inicio + velocidad × tiempo`), y en cada vuelta del lazo, cada ~500 µs **[código]**:

1. Actualiza el campo.
2. Lee el encoder. **Lee siempre**, porque el desenrollado necesita lecturas frecuentes para no perder períodos.
3. Registra solo **1 de cada 8 lecturas** (una muestra cada ~4 ms).

Cada muestra guardada es un par de números:

- **Entrada:** la lectura del encoder en **cuentas absolutas desenrolladas** (el contador crece de forma continua a lo largo de los períodos, no vuelve a cero en cada uno).
- **Salida deseada:** `motor_pos = (campo − campo_inicial) / 50`, en radianes. Es la posición de motor **comandada**.

Además se cuenta cuántas muestras tienen el aviso de campo débil, y durante el barrido se activa el chequeo de CRC (se restaura después). Salen del orden de 7000–8000 muestras en total (unas 4000 por pasada), a ~0,09° de motor entre muestras, lo que da unas 130 muestras por período del encoder.

**Diferencias con el barrido de caracterización:**

| Aspecto | `encoder_sweep` (G56) | Calibración (M56) |
|---|---|---|
| Firmware | De barrido | Real |
| Velocidad del campo | 78,5 rad/s (0,25 vueltas/s de motor) | 20 rad/s (0,064 vueltas/s) |
| Paso de actualización del campo | 100 µs | ~500 µs |
| Intervalo entre muestras | 1 ms | ~4 ms |
| Valor del encoder | Ángulo crudo del chip (0–2²¹) | Cuentas absolutas desenrolladas |
| Recorrido por sentido | 2 vueltas (720°) | 370° |
| Objetivo | Diagnóstico | Producir las tablas |

La velocidad distinta importa: el atraso del rotor depende de la velocidad, así que el error del barrido de caracterización y el de la calibración no son directamente comparables.

### 6.5 El archivo de calibración (`calib.csv`) y `capture_calibration.py`

**Para qué sirve el script.** El firmware arma las tablas dentro de la placa y no las envía a la PC. `capture_calibration.py` permite guardar las muestras en bruto que la calibración usó, para mirarlas y analizarlas después. Resuelve el problema de que la terminal se borra y no se pueden conservar los puntos **[notas]**.

```bash
python3 tools/capture_calibration.py --out calib
python3 tools/capture_calibration.py --from-log calib_log.txt   # analizar un log guardado
```

Manda `M56 P`, captura toda la salida (el programa corta cuando llega una línea `ok` o `error`) y genera:

| Archivo | Contenido |
|---|---|
| `calib.csv` | Las muestras, una por fila |
| `calib_log.txt` | El log completo de la placa |
| `calib.png` | 4 gráficos de diagnóstico |

**Columnas de `calib.csv` [código]:**

| Columna | Unidad | Origen | Qué representa |
|---|---|---|---|
| `motor_pos_rad` | rad | **Comandado**. `(campo − campo_inicial)/50`, calculado en la placa | Posición de motor "verdadera" que se usa como referencia. 0 es el home. Va de 0 a ~6,46 rad (370°) y vuelve |
| `field_angle_rad` | rad eléctricos | **Comandado** | Ángulo de campo ordenado. Es redundante: es lineal con la columna anterior (`campo_inicial + 50 × motor_pos`) |
| `encoder_raw` | cuentas | **Medido** | Cuentas absolutas desenrolladas del encoder. A diferencia del CSV del barrido, **no** está en el rango 0–2²¹: crece de forma continua. Las cuentas se convierten a `float`, por lo que a ~6,5·10⁷ cuentas el paso es de ~4 cuentas (0,0007° del sensor), un efecto despreciable |
| `weak_field` | 0/1 | **Medido** (chip) | Aviso de campo débil de esa muestra. Python lo toma de la línea de Teleplot `>weak_field:` que la placa imprime después de cada muestra. Overspeed y CRC no se guardan por muestra |

Las muestras de la ida y de la vuelta están una a continuación de la otra. El script las separa en el punto de mayor `motor_pos`.

**Qué analiza el script, para cada pasada [código]:**

1. Ajusta una recta `motor_pos = a·cuentas + b`. De la pendiente saca los **períodos de encoder por vuelta de motor**, que tienen que ser ~30.
2. Calcula el **residuo** respecto de la recta (RMS y máximo, en grados de motor). Ese residuo es justamente la no linealidad del sensor que la tabla va a corregir.
3. Cuenta los **pasos en los que el encoder va hacia atrás** (no debería haberlos).
4. Cuenta las muestras con campo débil.

**Cómo leer `calib.png` [notas]:** con todo sano, el panel 1 (encoder contra posición comandada) es una recta con la ida y la vuelta superpuestas, y el panel 4 (variación del encoder por muestra) no tiene picos grandes. Saltos de ±1 000 000 cuentas en el panel 4 son saltos de medio período (180° del sensor).

**Dos cosas a tener en cuenta:**

- El tiempo máximo de espera por defecto del script es 120 s (`--timeout`). `M56` puede tardar bastante más si el homing es largo, así que la captura se puede cortar antes de terminar. Conviene subir el valor.
- Al momento de escribir este documento no hay ningún `calib.csv` en la carpeta del proyecto. Todo lo que sigue sobre la calibración sale del código y de las notas, no de datos de calibración medidos.

### 6.6 Cómo se construyen las tablas

Con las ~8000 muestras, la placa arma las tablas en este orden **[código]**.

#### 6.6.1 Chequeo del campo débil

Si más del **5 %** de las muestras tienen aviso de campo débil, la calibración aborta con un mensaje que sugiere revisar la distancia entre el encoder y los imanes.

#### 6.6.2 Tabla principal: encoder → posición del motor

Es una tabla de 1024 puntos igualmente espaciados sobre el rango de cuentas medido (~30,8 períodos, ~6,5·10⁷ cuentas). Cada punto está separado del siguiente ~63 000 cuentas, o sea ~0,36° de motor (~2 µm de carro). Para cualquier lectura del encoder, el firmware busca entre qué dos puntos cae y **interpola linealmente**.

Hay que decidir qué valor poner en cada uno de los 1024 puntos. Eso se hace en dos etapas:

1. **Tabla inicial.** Se ordenan las muestras por lectura del encoder y se unen una a una por interpolación lineal. Es un punto de partida. Tiene dos problemas: pasa por cada muestra, así que copia su ruido, y como la ida y la vuelta pasan por las mismas cuentas pero con la posición comandada desplazada por el atraso del rotor, las muestras ordenadas se alternan entre las dos ramas y la curva oscila.

2. **Ajuste por mínimos cuadrados.** Se busca la tabla que mejor representa a *todas* las muestras a la vez. Se explica a continuación.

#### 6.6.3 El ajuste por mínimos cuadrados, en detalle

**Qué se ajusta y contra qué.** Se tienen N muestras `(x_i, y_i)`:

- `x_i`: lectura del encoder (cuentas absolutas).
- `y_i`: posición de motor comandada en ese instante (rad).

Las incógnitas son los 1024 valores `T_j` de la tabla, ubicados en los nodos `X_j`. Para una lectura `x` que cae entre los nodos `j` y `j+1`, la tabla predice:

```
ŷ(x) = (1 − α)·T_j + α·T_(j+1)        con  α = (x − X_j) / (X_(j+1) − X_j)
```

Es decir, un promedio ponderado de los dos nodos vecinos. Los pesos `(1−α)` y `α` son los que el código llama `w_a` y `w_b`.

El ajuste elige los valores `T` que minimizan la suma de los cuadrados de los errores:

```
S(T) = Σ_i ( ŷ(x_i) − y_i )²   +   λ · Σ_j ( T_j − T⁰_j )²
```

- El **primer término** es la distancia entre lo que predice la tabla y lo comandado, sumada sobre todas las muestras. Es lo que realmente se quiere minimizar.
- El **segundo término** tira de cada nodo hacia su valor en la tabla inicial `T⁰`, con un peso muy pequeño (`λ = 0,0001`). Solo importa en las celdas **sin muestras**, donde evita que el sistema quede indeterminado. En las celdas con datos casi no influye.

No es un ajuste a una recta ni a una fórmula. No se impone ninguna forma a la curva, salvo que sea lineal entre nodos. Los 1024 valores quedan determinados solo por los datos.

**Por qué esto cancela el atraso del rotor.** Un ejemplo con números inventados, en una celda donde la posición real es 5,00°:

| Muestra | Sentido | Posición comandada | Por qué |
|---|---|---|---|
| 1 | ida | 5,45° | En la ida, el comando va **adelantado** respecto de donde está el rotor |
| 2 | ida | 5,47° | idem |
| 3 | vuelta | 4,55° | En la vuelta, el comando va **atrasado** |
| 4 | vuelta | 4,53° | idem |

La interpolación directa entre muestras vecinas oscila entre ~5,5° y ~4,5°. El ajuste busca el valor que minimiza los cuadrados de las diferencias con las cuatro, que es su promedio: **5,00°**, la posición sin atraso. Además, promediar varias muestras reduce el ruido de cada una.

**Cómo se resuelve.** Cada muestra involucra como máximo a dos nodos vecinos. Por eso el sistema de ecuaciones (las "ecuaciones normales") tiene una estructura **tridiagonal**: cada nodo solo se relaciona con el anterior y el siguiente. Para cada nodo, la diagonal acumula la suma de los cuadrados de los pesos de las muestras cercanas, la posición vecina acumula el producto de los dos pesos, y el lado derecho acumula `peso × y`. Se resuelve con el algoritmo de Thomas, en un único recorrido de ida y vuelta, sin matrices grandes. Eso es lo que permite hacerlo dentro del microcontrolador.

**Cómo se evalúa la calidad del ajuste.** Después de ajustar, se calcula para cada muestra el residuo `tabla(x_i) − y_i`. A cada sentido se le **resta su media**, que es justamente el atraso del rotor (se informa aparte como "atraso del rotor"). Con lo que queda se calcula el RMS, que es el "error de la tabla". Ese número mide qué tan bien una única curva representa a las dos pasadas. Incluye el ruido del encoder, las irregularidades del micropaso y la diferencia de forma entre ida y vuelta. No separa cuánto aporta cada causa.

- Si el RMS supera **0,5°** de motor, la calibración aborta (la tabla se considera sospechosa).
- Un valor típico anterior a la reubicación del sensor fue de **0,13–0,15°** de motor (~0,8 µm de carro) **[notas]**.

El atraso medio típico es de 0,45–0,50° de motor a la velocidad de calibración **[notas]**.

#### 6.6.4 Chequeo de períodos enteros

Con la pendiente global de los datos se calcula cuántos períodos del encoder entran en una vuelta de motor. Como la rueda es un anillo cerrado, tiene que dar un **número entero** (30). Si difiere más de 0,2 del entero más cercano, el encoder perdió el seguimiento durante el barrido y la calibración aborta. Las notas mencionan valores de ~29,94 antes de ajustar la costura **[notas]**.

También se verifica que el rango medido cubra al menos una vuelta completa.

#### 6.6.5 Hacer la tabla periódica y corregir la costura

La tabla se midió sobre una sola vuelta, pero el husillo tiene unas 130. Como una vuelta de motor son exactamente 30 períodos, la tabla se declara **periódica**: fuera del rango medido, el firmware resta tantos períodos enteros como haga falta a la lectura, consulta la tabla, y suma a la salida una vuelta completa (2π) por cada período restado. Así, **una calibración de una vuelta cubre todo el recorrido**.

Aparece un problema: la pendiente medida sale un poco distinta de 30 exactos (cada pasada arranca desde parado y el atraso del rotor tarda en establecerse), pero la tabla se repite cada 30 exactos. En la unión de una vuelta con la siguiente (la "costura") quedaba un salto de ~0,5°. Como el home cae justo ahí, el ruido del encoder alrededor de la costura producía velocidades falsas enormes. Por eso el firmware mide el desfase de la costura, `tabla(inicio + período) − tabla(inicio) − 2π` (típico −0,44°), y lo **reparte linealmente sobre toda la vuelta**, para que la tabla empalme consigo misma **[código][notas]**.

#### 6.6.6 Segunda tabla: posición → ángulo de campo

Esta tabla **no se mide**. Por cómo se definió `motor_pos`, la relación es exacta y lineal:

```
campo = campo_inicial + motor_pos × 50
```

Por eso se construye como una recta de 2 puntos, que cubre ±1000 rad de motor (más que los ~817 rad del recorrido completo con el husillo de 2 mm). **[código]**

### 6.7 Guardado y carga desde la flash

Con `M56 S`, las dos tablas se guardan en el sistema de archivos de la placa (LittleFS) **[código]**:

| Archivo | Contenido | Tamaño |
|---|---|---|
| `axis_enc_to_pos_lut.dat` | Tabla encoder → posición | 4116 bytes |
| `axis_pos_to_field_lut.dat` | Tabla posición → campo | 28 bytes |

Formato binario de cada archivo: cantidad de puntos (entero de 4 bytes), mínimo y máximo de la entrada (dos `float`), los valores de la tabla (un `float` por punto), y los dos períodos (entrada y salida).

Al encender, el firmware busca estos archivos. Si los encuentra, carga las tablas y marca el eje como calibrado **sin validarlas**. Esos archivos viven en la memoria de la placa, no en la carpeta del proyecto, y no hay ningún comando que los vuelque a la PC.

**Advertencia importante [notas]:** la tabla depende de que el homing termine siempre en el mismo punto dentro del período del encoder. No está verificado que la tabla guardada siga siendo válida después de un reinicio. Mientras tanto, conviene correr `M56` en cada arranque. Para validar la tabla guardada: después de reiniciar, `G28` más la tabla guardada tiene que dar la misma posición que una calibración nueva.

### 6.8 Cómo se usa la tabla en el lazo cerrado

En cada ciclo del lazo (~22 kHz según las notas), el firmware hace **[código]**:

1. Lee el encoder y obtiene las cuentas absolutas.
2. La **tabla principal** las convierte en posición de motor (rad). El husillo la pasa a mm (2 mm por 2π rad).
3. Calcula el error de posición, `objetivo − posición`, y un PID de posición lo convierte en una velocidad objetivo.
4. Calcula la velocidad real a partir de la diferencia de **cuentas enteras** del encoder entre dos ciclos (no de la tabla, para evitar el ruido de la costura), con un filtro pasa bajos (constante de tiempo 4 ms).
5. Un PID de velocidad convierte la diferencia entre velocidad objetivo y real en un pequeño adelanto angular.
6. El ángulo de campo que se ordena al motor es `tabla posición→campo(posición) + adelanto`.

Hay una protección: si el error supera 1 mm durante más de 0,5 s, el motor se apaga y se activa una falla (`fault=1` en `M119`) **[código]**. Las ganancias de los PID vienen del proyecto original (para brazos livianos) y hay que reajustarlas con `M55`; el valor alto de `POS_KI` hace oscilar la velocidad **[notas]**.

### 6.9 Qué corrige la calibración y qué no

**Corrige:**

- La no linealidad repetible del encoder dentro de cada período.
- Las diferencias repetibles entre imanes y la excentricidad de la rueda, siempre que se repitan en cada vuelta (la tabla es de una vuelta).
- El atraso medio del rotor (se promedia entre ida y vuelta).

**No corrige:**

- Lo que no se repite: saltos de 180°, campo débil, patinaje de la rueda.
- Los errores del husillo (paso, juego): el encoder mide el motor, no el carro.
- Las irregularidades del micropaso del motor: la posición "verdadera" sale del campo comandado, así que esas irregularidades quedan guardadas en la tabla como si fueran correctas.

Para validar de verdad la exactitud, hay que calibrar o contrastar contra una referencia externa (interferómetro o encoder lineal). En el código hay un comentario sobre eso (`TODO: read motor_pos from precise reference encoder`).

---

## 7. Decisiones de diseño y resolución alcanzable

Esta sección responde preguntas que surgen al leer el código: por qué los valores son los que son, si se pueden cambiar y qué se ganaría.

### 7.1 ¿Por qué actualizar el campo cada 500 µs en la calibración?

El valor sale de `sleep_us(500)` en la función `rotate_field()`, heredada del proyecto original. **No hay una justificación documentada** en los archivos del proyecto. Lo que se puede decir es qué limita cada extremo **[código][estimación]**:

- **Límite superior.** El desenrollado del encoder pierde la cuenta si entre dos lecturas el ángulo avanza más de medio período (2²⁰ cuentas). A 20 rad/s de campo, el ángulo avanza ~2400 cuentas por paso de 500 µs. El margen es enorme.
- **Suavidad del motor.** En cada paso el campo avanza 0,01 rad eléctricos (0,57°). Pasos mucho mayores harían que el rotor se mueva a tirones.
- **Límite inferior.** Cada vuelta del lazo calcula un seno y un coseno, actualiza el PWM y lee por SPI. El barrido de caracterización demuestra que se puede ir a 100 µs. Pero acortar el paso **no agrega información**: lo que cuenta son las muestras que se registran, y esas ya se reducen a 1 de cada 8.

### 7.2 ¿Por qué se registra solo 1 de cada 8 lecturas?

Por memoria. Cada muestra ocupa 8 bytes (dos `float`), y con ~8000 muestras son ~70 KB de los 264 KB de RAM del microcontrolador. Sin diezmar no entrarían. Con `M56 P` hay además un costo de imprimir tres líneas por muestra. El código reserva el espacio de entrada para evitar que un redimensionamiento a mitad del barrido necesite el doble de memoria **[código]**.

### 7.3 ¿Por qué 1024 puntos en la tabla?

Es el valor por defecto de `ENCODER_LUT_SIZE` en `hw_config.h`, heredado del proyecto original, y da ~0,36° de motor por celda. No hay justificación documentada **[código]**.

Datos que condicionan esa decisión:

- **Memoria:** 4 bytes por punto. 1024 puntos son 4 KB. Es poco.
- **Velocidad de evaluación:** no depende del tamaño. El índice de la celda se calcula directamente a partir de la lectura, sin búsqueda **[código]**.
- **Cantidad de datos:** con ~4000 muestras por pasada y 1024 celdas, hay ~4 muestras por celda y por pasada, ~8 entre ida y vuelta.

### 7.4 ¿Se puede aumentar? ¿Daría más resolución?

**Se puede aumentar**: es una constante (`constexpr`) y el archivo guardado incluye el tamaño. **No da más resolución**, por dos razones:

1. **La resolución no depende de la tabla.** La tabla interpola linealmente entre celdas, así que devuelve valores continuos. La resolución la fijan el encoder y los números de punto flotante (ver 7.5).
2. **Más celdas solo cambian cuán fielmente la tabla copia la forma de la no linealidad.** El error de interpolar una onda sinusoidal de amplitud `A` y período `P` con celdas de ancho `h` es aproximadamente:

```
error de interpolación ≈ A · (1 − cos(π·h / P))
```

Con celdas de 0,36° y una ondulación de ±20° del sensor (±0,67° de motor) **[estimación]**:

| Período de la ondulación | Error de interpolación |
|---|---|
| 12° de motor (el período del encoder) | ~0,003° de motor ≈ **17 nm** de carro |
| 6° de motor (medio período, la armónica que describen las notas) | ~0,012° de motor ≈ **66 nm** de carro |

Esas decenas de nanómetros son mucho menores que el residuo actual de la tabla (~0,8 µm). Mientras el residuo domine, agrandar la tabla no mejora nada.

Además, aumentar sin más datos **empeora**: con 4096 celdas y ~8000 muestras quedan ~2 muestras por celda y la tabla copia más ruido, porque el ajuste no tiene ningún término de suavizado entre celdas vecinas.

**Lo que sí mejoraría la calibración:**

- Promediar **más pasadas** (el ruido baja con la raíz de la cantidad de muestras por celda).
- Una **velocidad más lenta** o menos diezmado (más muestras por celda).
- **Acumular las ecuaciones normales en línea**: como son sumas por celda, no hace falta guardar las muestras, y la memoria deja de limitar el barrido. **[estimación]**
- Una **referencia externa** (interferómetro). Es la mejora con mayor impacto, porque hoy la "verdad" es el campo comandado.

### 7.5 ¿Cuál es la mayor resolución posible?

Resolución y exactitud son cosas distintas. La **resolución** es el escalón más pequeño que se puede leer u ordenar. La **exactitud** es cuánto se parece lo leído a la posición real. Esta tabla da la resolución de cada eslabón, convertida a mm de carro:

| Eslabón | Resolución en el carro | De dónde sale |
|---|---|---|
| Cuenta del MT6835 | 0,032 nm | 66,7 µm por período / 2²¹ |
| Ruido del encoder en reposo | ~3 nm | ~100 cuentas con BW = 5 según las notas, medidas con el sensor mal ubicado y sin aclarar si es pico a pico o RMS. **Hay que volver a medirlo** **[notas][estimación]** |
| Cuantización del campo ordenado (PWM) | ~2,6 nm por nivel | PWM de 12 bits (4095) por corriente 0,6 da 2457 niveles; ~4·10⁻⁴ rad eléctricos por nivel **[código][estimación]** |
| `motor_pos` en `float` de 32 bits | ~2 nm a 30 mm, ~10 nm a 130 mm, ~19 nm a 255 mm | El servo guarda `motor_pos` y el error de posición en `float` y llegan a ~817 rad. El paso de un `float` crece con su valor **[código][estimación]** |
| Cuentas absolutas convertidas a `float` al consultar la tabla | ~16 nm al final del recorrido | Por encima de 4,3·10⁹ cuentas el paso es de 512 cuentas **[código][estimación]** |
| Posición en mm (`float`) | ~15 nm a 255 mm | **[estimación]** |

**Conclusión sobre resolución.** El piso numérico del firmware es de unos pocos nm cerca del home y de unos 20 nm al final del recorrido. Es un límite de software y se puede bajar, por ejemplo manteniendo la posición como cuentas enteras de 64 bits más un resto pequeño en `float`. La velocidad ya se calcula con cuentas enteras por esta razón **[notas]**. El motor real, con fricción y par de retención, tampoco resuelve pasos de nanómetros.

**Conclusión sobre exactitud.** Hoy lo que limita es otra cosa:

- Sin corregir, el error del sensor contra el campo comandado es de ~2 µm RMS y ~6 µm de máximo **[medido]**.
- La tabla tiene celdas de ~2 µm y dejó un residuo de ~0,8 µm RMS antes de reubicar el sensor **[notas]**.
- El encoder mide el motor y no el carro: el paso y el juego del husillo no se ven.
- La referencia de la calibración es el campo comandado, con sus irregularidades.

Un orden de magnitud realista **[estimación]**: lectura y orden de posición en decenas de nm, y exactitud del orden de 1 µm sin referencia externa. Bajar de ahí exige calibrar la tabla contra el interferómetro, y probablemente una tabla a lo largo del husillo y no solo por vuelta.

---

## 8. Puntos de atención y recomendaciones

1. **`x_mm` no es la posición del carro.** Es el arco sobre la rueda de imanes. El carro avanza `motor_deg/360 × 2 mm`. Cualquier gráfico del CSV contra `x_mm` está en milímetros de rueda.
2. **El CSV de `encoder_sweep` no guarda los parámetros de la corrida.** La línea `META` solo se imprime en consola. Si `--magnets`, `--pitch` o `--pole-pairs` no coinciden con el montaje, `motor_deg`, `x_mm` e `ideal_deg` salen mal sin ningún aviso. Conviene guardar `META` junto a cada CSV, y anotar la posición del sensor y la corriente usada.
3. **`capture_calibration.py` tiene un tiempo de espera de 120 s**, menos que lo que puede tardar `M56` con el homing. Subir `--timeout`.
4. **La referencia de ambos procesos es el campo comandado**, no una medición independiente (sección 2.3). Los números de este documento son relativos a esa referencia.
5. **Las velocidades del barrido y de la calibración son distintas** (0,25 contra 0,064 vueltas/s), por lo que el atraso del rotor y el error no son directamente comparables.
6. **Los valores de la tabla anteriores a la reubicación del sensor** (residuo de 0,13–0,15°, ~4 % de campo débil, zona débil en ~212–219°) se midieron con el sensor mal orientado **[notas]**. Todavía no hay una calibración hecha con el sensor en la posición buena.
7. **La calibración guardada en flash (`M56 S`) no está validada** tras un reinicio. Correr `M56` en cada arranque hasta verificarla.
8. **El homing contra el otro tope (`D1`) no funciona** todavía **[notas]**.
9. **El montaje del sensor en la mejor corrida no está confirmado.** La de las 18:13 fue sostenida a mano; falta confirmar cómo estaba la de las 18:21 **[notas]**. Hace falta un soporte rígido que reproduzca esa posición: el chip pasando la cara de los imanes entre 1 y 1,5 mm, y lo más cerca posible en dirección radial. **[notas]**
10. **Tornillos de acero cerca del sensor.** Las notas recomiendan cambiarlos por tornillos no magnéticos y comparar con un barrido **[notas]**.
11. **Excentricidad de la rueda.** La variación del error por sector (6° contra 18° del sensor) sugiere que queda alguna. Se puede medir con reloj comparador o calibres de láminas **[notas]**.
12. **El nombre `G56` / `M56`** puede confundir (sección 3.2).
13. **Hay un cuelgue de la placa sin explicar** (no respondía por serie ni al reinicio por software). Investigar si vuelve a pasar **[notas]**.

---

## 9. Apéndices

### 9.1 Archivos del repositorio relacionados

| Archivo | Para qué sirve |
|---|---|
| `tools/encoder_sweep.py` | Captura y análisis del barrido de caracterización |
| `tools/plot_periods.py` | Análisis por sectores de un CSV de barrido |
| `tools/capture_calibration.py` | Captura y análisis de los datos de `M56 P` |
| `tools/send_gcode.py` | Envío de comandos en secuencia |
| `src/main.cpp` | **Firmware de barrido** (temporal): comando `G56` |
| `main_firmware_completo.cpp.bak` | Respaldo del **firmware real** |
| `src/hardware/MT6835_encoder.cpp/.h` | Driver del encoder: lectura SPI, CRC, estado, desenrollado |
| `src/hardware/TB6612_motor_driver.cpp/.h` | Driver del motor: seno/coseno por PWM, `rotate_field` |
| `src/servo_control/actuator_calibration.cpp/.h` | Medición de la calibración y armado de las tablas |
| `src/utilities/lookup_table.cpp/.h` | La tabla: interpolación, mínimos cuadrados, periodicidad, guardado |
| `src/servo_control/servo_controller.cpp` | Lazo cerrado: usa las tablas en cada ciclo |
| `src/servo_control/homing_controller.cpp` | Homing contra el tope |
| `src/axis_controller.cpp` | Orquesta homing, calibración, guardado y falla |
| `src/command_processor.cpp` | Interpreta `M56`, `G28`, `G1`, `M17`, etc. |
| `src/hw_config.h` | Parámetros del montaje y de la calibración |
| `mediciones/` | CSV y PNG de los barridos |
| `SESION_2026-09-29.md` | Resumen de la sesión de caracterización |
| `HISTORIAL_DE_CAMBIOS_FIRMWARE.md` | Detalle de cada cambio del firmware |
| `SIMULACIONES_CLEAN.ipynb` | Simulación del campo magnético (magpylib) |

### 9.2 Fórmulas de conversión

```
Grados de motor   = grados del sensor / 30
µm de carro       = grados del sensor × 0,185         (2000 µm / 30 períodos / 360°)
µm de carro       = grados de motor × 5,556           (2000 µm / 360°)
mm de carro       = rad de motor × 2 / (2π)
rad eléctricos    = rad de motor × 50
Cuentas → grados del sensor = cuentas / 2²¹ × 360
Períodos por vuelta         = |d(cuentas)/d(rad de motor)| × 2π / 2²¹   → debe dar ~30
```

### 9.3 Flujo de trabajo propuesto

Basado en los próximos pasos de `SESION_2026-09-29.md` **[notas]**:

1. **Fijar el sensor** en la posición buena con un soporte rígido.
2. **Verificar con un barrido** (`encoder_sweep.py`): tiene que dar 0 % de campo débil, 0 saltos y ~30 períodos por vuelta. Opcional: `plot_periods.py` para ver los sectores.
3. **Volver al firmware real** (`cp main_firmware_completo.cpp.bak src/main.cpp`, `pio run -t upload`).
4. **Calibrar con `M56`** (o `capture_calibration.py` para guardar los datos). Debe terminar en `calibracion OK`, sin aviso de campo débil y con error de la tabla muy por debajo de 0,5°.
5. **Probar el lazo cerrado:** `M17`, luego `G1 X2 F0.3`, `G1 X10` y `G1 X50`, con un `M119` después de cada uno. Debe quedar `fault=0` y la posición cerca del objetivo. Si algo se traba, `M121` inmediatamente después.
6. **Ajustar el PID** con `M55`.
7. **Validar** la calibración guardada en flash (`M56 S`) tras un reinicio.
8. **Contrastar con el interferómetro**, que es la validación independiente del proyecto.

### 9.4 Preguntas abiertas

- ¿Cuál es el ruido real del encoder en reposo con el sensor bien ubicado y BW = 5?
- ¿Qué tan repetible es la tabla entre calibraciones sucesivas?
- ¿Cuánto del error del barrido (~2 µm RMS) es del encoder y cuánto del accionamiento? Solo se separa con una referencia externa.
- ¿Cómo afecta la velocidad de calibración al atraso del rotor y al residuo de la tabla?

---

*Documento generado por Claude el 01/10/2026 a partir de la lectura del firmware y los scripts del proyecto y del análisis del archivo `encoder_sweep_20260929_182151.csv`. Las cifras marcadas como [notas] no fueron verificadas de forma independiente. No se dispuso de ningún archivo de calibración (`calib.csv` ni las tablas guardadas en la flash).*


