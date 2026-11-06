# 2. Desarrollo experimental

Material para la sección de desarrollo experimental: qué se construyó, cómo se diseñó la parte crítica (la rueda de imanes), cómo funciona el firmware y la calibración, y cómo se hizo cada ensayo. Entre corchetes, la fuente.

---

## 2.1 Montaje mecánico

| Elemento | Dato | Fuente |
|---|---|---|
| Guía | Carro sobre guía de rodillos cruzados (en el borrador de publicación: sobre una placa de granito) | [README master, SHOWCASE] |
| Transmisión | Husillo de bolas, **5 mm por vuelta** (medido el 02/10 moviendo el carro a mano con el motor apagado y comparando calibre contra `M114`) | [cuaderno 2/10] |
| Recorrido | ~260 mm entre topes; el firmware acepta 0–255 mm | [`hw_config.h`] |
| Referencia de posición | Tope mecánico del lado de home. No hay finales de carrera: el homing detecta el tope porque el encoder deja de seguir al campo | [cuaderno 2/10] |
| Motor | Paso a paso híbrido de 1,8°, 50 pares de polos, doble eje (el eje trasero lleva la rueda de imanes). R = 33 Ω, L = 37 mH, K_t = 0,46 N·m/A | [cuaderno 12/9; PLAN_DE_PRUEBAS_MODULAR] |
| Acople motor–husillo | Rígido (según el plan de pruebas). Se desacopló del 30/10 al 03/11 para ensayar el motor solo, y se reacopló el 03/11 | [PLAN_DE_PRUEBAS_MODULAR; rama `prueba/sin-acople`] |
| Rueda de imanes ("motor horn") | Impresa en 3D (diseño propio en Onshape), buje con prisionero, 60 alojamientos en la circunferencia, tapa a presión para no pegar los imanes. Radio del anillo 28,9 mm, paso entre imanes 3 mm. Pegada al eje el 29/9 porque se deslizaba | [cuaderno 25/8, 1/9, 12/9, 29/9] |
| Imanes | 60 cilindros de NdFeB **Ø3 × 5 mm, N35**, magnetizados axialmente, polaridad alternada → 30 pares de polos | [cuaderno 18/8; README] |
| Soporte del encoder | Pieza impresa con sujeción ajustable. En el montaje actual el sensor queda en un voladizo de 56 mm con ~0,3 mm de juego radial | [cuaderno 1/9; SHOWCASE §3] |

**Orientación del sensor (29/9).** El encoder se había montado con la placa paralela a la cara de la rueda, de modo que el chip medía el campo en el plano XY (radial–tangencial), donde es débil y deformado: 10–18 % de lecturas con campo débil, cientos de saltos de 180°, 15–22 períodos por vuelta en lugar de 30. Se reorientó como en el diseño original: placa parada, tangente al borde de la rueda y paralela al eje de los imanes, midiendo en el plano XZ (tangencial–axial) de las simulaciones. Con el modelo: en XZ el error de ángulo en el óptimo es 0° con 58 mT; en XY, 14°, y hasta 98° si el chip queda más centrado sobre la fila [cuaderno 29/9].

**Posición del chip.** Tabla calculada con el modelo (imanes 3 × 5 mm N35, paso 3 mm). Distancia radial desde el eje de los imanes (radio 1,5 mm); altura desde el plano medio (−2,5 mm es la cara). Cada celda: campo / error de ángulo [cuaderno 29/9].

| Radial \ Altura | 0 mm | −2,5 mm (cara) | −3,5 mm | −4,05 mm | −5,0 mm |
|---|---|---|---|---|---|
| 1,6 mm | 0 mT / 156° | 2 mT / 91° | 97 mT / 2° | **57 mT / 0,1°** | 21 mT / 1° |
| 2,0 mm | 0 / 156° | 1 / 87° | 55 / 8° | 38 / 4° | 16 / 1° |
| 2,5 mm | 0 / 156° | 1 / 83° | 25 / 17° | 21 / 9° | 11 / 4° |
| 3,0 mm | 0 / 156° | 1 / 80° | 11 / 24° | 11 / 14° | 7 / 7° |
| 3,5 mm | 0 / 156° | 1 / 77° | 5 / 30° | 6 / 19° | 4 / 10° |

Regla práctica: el chip tiene que quedar **entre 1 y 1,5 mm más allá de la cara de los imanes y lo más cerca posible en dirección radial**. El montaje real no está en el óptimo: el borrador de publicación indica que el campo en el chip queda cerca del mínimo de 30 mT.

---

## 2.2 Electrónica

| Componente | Dato | Fuente |
|---|---|---|
| Microcontrolador | Raspberry Pi Pico (RP2040, 2 núcleos Cortex-M0+, sin unidad de punto flotante: `float` y `double` por software), framework Arduino-Pico + Pico SDK, PlatformIO | [README] |
| Driver | TB6612FNG, doble puente H; 15 V máx., 1,2 A continuos por canal. PWM aplicado en las entradas `INx` (decaimiento lento), `PWMx` en alto. Pines: AIN1 GP2, AIN2 GP3, PWMA GP4, BIN1 GP5, BIN2 GP6, PWMB GP7, STBY GP8 | [cuaderno 21/8; README master] |
| Encoder | MT6835, 21 bits, SPI a 8 MHz modo 3 (CS GP17, SCK GP18, MOSI GP19, MISO GP20). Ancho de banda interno BW = 5, histéresis 4. Con BW 6 y 7 baja el overspeed pero el ruido en reposo sube de ~100 a >5800 cuentas | [cuaderno 29/9; RESULTADOS 06/11] |
| Fuente | Fuente de laboratorio. 10 V / 1 A en septiembre; 12 V en las pruebas del 30/10. **Confirmar con qué tensión se hicieron las mediciones finales** | [SESION_2026-09-29; commits 30/10] |
| Comunicación | USB CDC a 921 600 baudios: comandos G-code en texto y telemetría binaria intercalada | [README] |

---

## 2.3 Diseño de la rueda de imanes (simulación)

Notebook: `simulaciones/SIMULACIONES_CLEAN.ipynb` (esta rama) y `simulations/magnet_wheel.ipynb` (versión limpia en `origin/master`). Librería **magpylib** (cálculo analítico del campo de imanes con magnetización homogénea).

**Modelo.** La rueda se "desenrolla" en una fila recta de imanes cilíndricos con polaridad alternada. Primero 15 imanes; después 30, usando solo la zona central para que el campo se comporte como el de un anillo completo, sin efectos de borde [cuaderno 14/8].

**Procedimiento** (en el orden en que se hizo, cuaderno 11/8 a 18/8):

1. Campo de un imán y de la fila en el plano XZ; gráfico de flechas normalizado con la intensidad en color (log₁₀|B|).
2. Ángulo del campo `arctan2(B_z, B_x)` a lo largo de X para distintas alturas Z: en Z = 0 es una onda cuadrada (campo puramente axial); entre ~2 y ~12 mm es aproximadamente lineal.
3. Barrido en Y (distancia lateral) con Z fijo, mirando linealidad y circularidad del Lissajous: más cerca, mejor.
4. Barrido en Z con Y = 3 mm: R² > 0,99 entre Z = 3 y 8 mm, pero |B| > 30 mT solo hasta Z ≈ 4,5 mm. **Lo que acota la ventana es el campo mínimo del sensor, no la linealidad.**
5. Criterio unificado: mapa del CV de dθ/dx (ecuaciones 1.4 y 1.5) sobre todo el plano (Y, Z), descartando los puntos dentro del imán o con |B| < 30 mT.
6. Comparación de geometrías (Ø5 × 3 mm contra Ø3 × 5 mm) y de grados (N35 contra N52).

**Resultados del diseño:**

| Configuración | Óptimo (Y, Z) | CV | \|B\| en el óptimo | Fuente |
|---|---|---|---|---|
| Ø3 × 5 mm, N35, paso 3 mm | Y = 1,59 mm del eje (en el borde del imán), Z = 4,05 mm (1,55 mm más allá de la cara) | 0,17 % | 58 mT | [README `origin/master`; cuaderno 18/8] |
| Ø3 × 5 mm, N52 | mismo punto | mismo CV | 70 mT | [cuaderno 18/8] |
| Ø5 × 3 mm, paso 5 mm | — | 0,035 % | ~15 mT | [README `origin/master`] |

En el óptimo el ángulo es una recta en cada período de 6 mm sobre la rueda (R² = 1,0000) y el Lissajous es un círculo.

**Elección: Ø3 × 5 mm, N35.** Entran más imanes en la circunferencia (más pares de polos, más resolución), el ángulo es casi tan lineal como con Ø5 × 3 y el campo es cuatro veces mayor. El N35 es el que se consigue en el mercado local; el N52 solo daría más margen de campo.

> Hay dos versiones de la tabla de óptimos en el cuaderno. La del 14/8 (Y = 2–3 mm, Z = 4,2–4,5 mm, CV 2,8–5,9·10⁻², ~30 mT) es de un barrido con la región Y restringida. La final es la del 18/8 (Y = 1,59, Z = 4,05). Usar la del 18/8.

---

## 2.4 Firmware

### Arquitectura

```
 Núcleo 0                                        Núcleo 1
 ────────                                        ────────
 USB serie ─► parser G-code ─► comandos          encoder (SPI) ─► tabla de calibración ─► posición
 temporizador 2 kHz ─► perfil trapezoidal ───►   PI posición ─► PI velocidad ─► ángulo de campo
                 objetivo en int32 (0,25 nm)     conmutación FOC en el TB6612
 serie ◄─ telemetría (cola circular) ◄───────── estado del lazo
```

- **Núcleo 1**: lazo servo en un bucle sin interrupciones de comunicación, a **~16 kHz** (15,9 vueltas/ms, 62,8 µs por vuelta). El reloj del sistema está en 250 MHz.
- **Núcleo 0**: comandos, perfil trapezoidal (aceleración 5 mm/s², crucero hasta 2 mm/s por defecto) en una interrupción de 2 kHz, y vaciado de la telemetría.
- **Datos compartidos** protegidos con un *spinlock* de hardware; la interrupción usa `spin_try_lock` y, si está ocupado, saltea ese ciclo. La posición pasa entre núcleos como un `int32` en unidades de 0,25 nm (una escritura atómica).

### Qué se reutilizó y qué se escribió

- **Del proyecto original, sin cambios o con cambios menores:** drivers del MT6835 y del TB6612, PID, homing, parser de G-code, registro de mensajes.
- **Extendido:** controlador servo (velocidad desde la tabla, realimentación anticipada, anti-cogging, corte por falla), calibración (barrido de una vuelta completa, plegado, filtro de armónicos) y tabla de búsqueda (periódica, evaluación con vueltas enteras en aritmética entera).
- **Nuevo:** perfil trapezoidal (reemplaza el planificador y la cinemática del delta), controlador de eje (mm ↔ rad, homing, calibración, verificación de conmutación), procesador de comandos, aprendizaje del anti-cogging, telemetría, `main.cpp`.

### Representación numérica de la posición (23/10)

Con `float` de 32 bits (24 bits de mantisa) la posición en radianes de motor tenía pasos de 3 nm a 30 mm y de 24 nm cerca de 255 mm: en la práctica el lazo tenía un sensor de 24 nm al final del recorrido. Se pasó a: vueltas enteras restadas en aritmética entera de 64 bits antes de consultar la tabla, posición, objetivo y error en `double`, conmutación en `float`. El lazo mantuvo su velocidad (15,9 vueltas/ms contra 16,0) [docs/CAMBIOS_2026-10-23_PRECISION.md].

### Protecciones

- Corte por falla si |error| > 1 mm durante 0,5 s; guarda una traza de ~2 s del lazo (`M121`).
- El homing aborta si el encoder no sigue al motor (rueda floja).
- `M17` exige homing y calibración. Después de un corte, rearma quieto en la posición actual.

### Telemetría

Paquete binario de 28 bytes con encabezado `0xAA 0x56`: tiempo (µs), objetivo y posición (int32, 0,25 nm), velocidad, salida del lazo, ángulo crudo del chip, bits de estado, banderas y contador para detectar pérdidas. Se toma en la interrupción de 2 kHz del núcleo 0 (`M120 S1 P2` → 1 kHz). Se pierde ~2,5 % de los paquetes (de a uno). Importante para el análisis: el objetivo es el recién calculado y la posición es la última publicada por el núcleo 1, así que hay un desfasaje de ~0,5 ms que aparece como **±0,7 µm de atraso aparente a 2 mm/s** en todas las configuraciones [REPORTE 16/10].

---

## 2.5 Calibración (`M56`)

Duración total ~3–5 min. Se corre **en cada arranque** (la tabla guardada en flash puede no corresponder a la mecánica; ver resultados 3.3).

1. **Homing.** En lazo abierto, a 2 rad/s de motor (1,6 mm/s) y corriente 0,6, contra el tope. Se declara el tope cuando el encoder avanza menos de la mitad que el campo en 3 ventanas seguidas de un período. El contador del encoder se fija módulo un período.
2. **Barrido.** El campo gira en lazo abierto **400° de motor** hacia adelante y 400° hacia atrás a 20 rad/s eléctricos (0,064 vueltas/s, 0,32 mm/s de carro). Una muestra cada ~4 ms: par (cuentas absolutas del encoder, `θ_m` comandado = campo/50).
3. **Descarte de puntas.** Se descartan 15° en cada extremo, donde el atraso del rotor todavía no se estableció, y se usa solo el rango cubierto por las dos pasadas.
4. **Plegado y ajuste.** Las muestras se pliegan sobre una vuelta de motor (30 períodos exactos) y se ajusta una **tabla periódica de 1024 nodos** (~0,35° de motor, ~4,9 µm de carro por celda) con interpolación lineal, por mínimos cuadrados:
   ```
   S(T) = Σ_i ( ŷ(x_i) − y_i )²  +  λ Σ_j ( T_j − T⁰_j )²,   λ = 10⁻⁴        (2.1)
   ```
   Cada muestra involucra dos nodos vecinos, así que las ecuaciones normales son tridiagonales y se resuelven con el algoritmo de Thomas dentro del microcontrolador (verificado en la PC contra `numpy.linalg.lstsq`). Como en cada nodo hay muestras de ida (comando adelantado) y de vuelta (comando atrasado), el ajuste queda en el promedio y **cancela el atraso del rotor**.
5. **Controles.** Aborta si más del 5 % de las muestras tiene campo débil, si el RMS del ajuste supera 0,5° de motor, o si los períodos por vuelta no dan 30 (con tolerancia 0,2).
6. **Filtro de la huella del motor.** Resta de la tabla los armónicos 50·m por vuelta (con fase en la posición del motor), salvo los que coinciden con armónicos de la rueda (150, 300, 450…).
7. **Tabla posición → campo.** No se mide: es la recta `campo = campo₀ + 50·θ_m`.
8. **Verificación de conmutación.** Gira el campo 2 ciclos eléctricos de ida y vuelta y compara el campo comandado con el que da la tabla, con el contador tal cual, corrido −1 y +1 período. Con el contador correcto el desfase es ~0; corrido, ~±120°. Si el mejor es < 45° y el contador estaba corrido, lo corrige; si ninguno baja de 45°, falla. (Un período de encoder son 12° de motor = 600° eléctricos ≡ 240°: si el homing cae del otro lado del borde del período, la conmutación queda 120° o 240° corrida y el motor empuja al revés.)
9. **Reaprendizaje del anti-cogging.** Dos idas y vueltas 0 → 50 → 0 mm a 2 mm/s en lazo cerrado; se pliega el error de seguimiento sobre el ciclo eléctrico (64 intervalos, por sentido) y se corrige la tabla de adelanto con un paso de Newton usando respuestas armónicas medidas el 20/10. Queda la tabla de la pasada con menos RMS.
10. Con `M56 S`, guarda tablas y anti-cogging en la flash (LittleFS, con el núcleo 1 pausado con `multicore_lockout`).

**Anti-cogging** (cómo actúa): `δ = clamp(salida_PI + G·[T(φ) + s·T_s(φ)], ±81°)`, con `φ` el ángulo eléctrico, `T` la parte igual en los dos sentidos, `T_s` la que cambia de signo con el sentido y `s` = velocidad del perfil / 2 mm/s (limitado a ±1). Por encima de 2 mm/s la compensación se desvanece linealmente hasta 0 en 3 mm/s.

---

## 2.6 Ensayos

| Ensayo | Qué mide | Cómo | Herramienta | Datos |
|---|---|---|---|---|
| Barrido de caracterización del encoder | No linealidad del ángulo crudo, avisos del chip, períodos por vuelta | Firmware aparte; campo en lazo abierto 2 vueltas por sentido a 0,25 vueltas/s, muestra cada 1 ms; se compara con un encoder ideal (recta con un desfase por sentido) | `G56`, `tools/encoder_sweep.py`, `tools/plot_periods.py` | `mediciones/2026-10-13/encoder_sweep_*`, `2026-11-03/encoder_sweep_*` |
| Medición de la conmutación | Desfase entre campo comandado y el que da la tabla, a lo largo del recorrido | Lazo detenido, campo en lazo abierto ida y vuelta; promedio de sentidos = error de conmutación; semidiferencia = atraso por fricción | `M57 X<mm>` | `mediciones/2026-10-13/diagnostico_vuelta/`, `2026-11-06/m57_*.log` |
| Ida y vuelta con telemetría | Error de seguimiento en movimiento y en reposo, salida del lazo, saturación, estado del encoder | `G1` entre 0 y 50 o 255 mm a 2 mm/s (1 kHz); resumen por tramo y gráficos | `tools/closed_loop_test.py` | `mediciones/*/lazo_cerrado_*` |
| Barridos de ganancias | Efecto de cada ganancia en movimiento y en reposo | Una calibración; ganancias cambiadas en caliente con `M55`; idas y vueltas con reposos de 10 s | `tools/pid_sweep.py`, `tools/ki_eleccion.py`, `tools/ki_sweep.py` | `mediciones/2026-10-16/barrido_*`, `2026-10-20/barrido_pid_*` |
| Intervención B1/A1 | Causa del atraso y de la ondulación por imán | Las 4 combinaciones de `M58 V<0|1> F<0|1>` con la misma calibración, carril completo | `closed_loop_test.py` | `mediciones/2026-10-16/lazo_cerrado_*_V?F?` |
| Aprendizaje del anti-cogging | Convergencia de la compensación | Iteraciones de ida y vuelta 0 → 50 → 0 mm, secante armónico por armónico | `tools/cogging_learn.py` | `mediciones/2026-10-20/cogging_*` |
| Escalera nanométrica | Ruido en reposo y menor paso resuelto | A 30 mm: 5–10 s quieto y 5 pasos absolutos de 10 nm de 4 s cada uno; telemetría a 1 kHz; se descarta el primer segundo de cada escalón | `tools/escalera_nm.py`, `tools/plot_escalera.py`, `informe/scripts/resumen_escaleras.py` | `mediciones/2026-11-06/escalera_*.csv` |
| Ruido del encoder con el motor apagado | Ruido propio del sensor | `M18` y telemetría | — | `mediciones/2026-11-06/reposo_ganancias/off_P*.npz` |
| Barrido de `VEL_KP` en reposo | Origen de la oscilación de ~214 Hz | Reposo a 30 mm, ganancias con `M55`, espectro | — | `mediciones/2026-11-06/reposo_ganancias/` |
| Velocidad | Error y margen de torque a 2–5 mm/s | 0 → 50 → 0 mm con `M203 X2…5` | `tools/prueba_velocidad_acoplado.py` | `mediciones/2026-11-03/lazo_cerrado_*_acoplado_v*` |
| Motor sin acople (30/10–03/11) | Techo de velocidad en lazo abierto, torque, fijación de la rueda | `M61`, `M62`, `M63`, rampas hasta perder el paso | `tools/prueba_torque.py`, `prueba_velocidad.py`, `prueba_rueda.py` (rama `prueba/sin-acople`) | `git show prueba/sin-acople:mediciones/2026-10-30/…`, `…/2026-11-03/RESULTADOS.md` |

**Procedimiento común:** `M56` al empezar (anotar RMS de la tabla, desfase de conmutación y RMS del anti-cogging), `M17`, ensayo, guardar CSV/NPZ y resumen en `mediciones/AAAA-MM-DD/`.

---

## 2.7 Análisis: plegado y presupuesto del error

**Plegado (promedio sincrónico).** Para una longitud P se calcula la fase de cada muestra (`x mod P`) y se promedian las muestras que caen en la misma fase. Lo que se repite con P se suma en fase; el ruido se divide por √N; lo que se repite con otra longitud (no múltiplo) se cancela. Con ~127 000 muestras de crucero por ida y 36 intervalos de fase, un ruido de 2 µm queda en 0,03 µm. El método es muy selectivo: un período corrido δ desfasa L·δ/P a lo largo de L = 255 mm, así que el pico tiene un ancho de ~P²/L ≈ 0,03 µm. Por eso **los períodos no se eligen a mano**: un barrido de P entre 70 y 105 µm muestra picos solo en 75 µm (3 pasos), 83,3 µm (imán) y 100 µm (ciclo eléctrico) [docs/METODO_DE_ERRORES, fig. 7].

**Presupuesto** (`tools/analisis_errores.py`, función `budget()`), sobre el crucero de cada tramo y en este orden, restando cada componente antes de la siguiente:

1. constante (media);
2. por imán (plegado sobre el ángulo crudo módulo 180°, que es la fase física exacta del imán);
3. por paso (25 µm) — antes que el ciclo eléctrico, porque 100 µm = 4 × 25 µm;
4. por ciclo eléctrico (100 µm);
5. por vuelta (5 mm), al final porque contiene a todas;
6. resto (desvío estándar de lo que queda).

La amplitud informada es la mitad del pico a pico de la forma media.

**Atribución.** El plegado dice qué parte del error se repite con cada longitud; la causa se estableció con (a) predicciones cuantitativas desde el código antes de medir, (b) experimentos de intervención cambiando una sola cosa por vez y (c) repetibilidad y signo con el sentido de marcha.

**Reposo.** Desvío estándar de la posición sin el primer segundo después de llegar; fracción de potencia en bandas del espectro (por ejemplo 150–260 Hz para la oscilación del lazo de velocidad).
