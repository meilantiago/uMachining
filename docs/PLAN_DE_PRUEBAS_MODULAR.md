# Plan de pruebas modular del eje X

**Versión:** 09/10/2026. Reemplaza a [PLAN_DE_PRUEBAS.md](PLAN_DE_PRUEBAS.md), que es de antes de tener el hardware.
**Objetivo:** que el eje sea confiable (sin cortes, sin deriva, con velocidad y rizado conocidos) antes de montar el interferómetro.
**Contexto:** [SESION_2026-10-09.md](SESION_2026-10-09.md) (datos del motor y análisis de torque) y [CAMBIOS_2026-10-06.md](CAMBIOS_2026-10-06.md) (cortes y conmutación desfasada).

---

## 1. Idea general

Cuando hoy se corta el lazo, la falla puede venir de cinco lugares a la vez: el encoder, el motor con su driver y su fuente, la mecánica, el lazo de velocidad o el lazo de posición. Por eso cuesta tanto diagnosticar.

El plan **aísla cada eslabón de la cadena con una prueba propia**, con un criterio numérico de "pasa / no pasa", y no se sube al eslabón siguiente hasta que el anterior pasa. No se desarma el firmware: las pruebas son **comandos de diagnóstico dentro del firmware real**, más un script de PC por prueba que decide si pasa. Lo que ya funciona no se toca.

```
Nivel 0  Banco: fuente, bobinas, marcas, integridad del SPI
   │
Nivel 1  Encoder: ruido, calibración, conteo de períodos          (sin PID)
   │
Nivel 2  Motor + driver + fuente + mecánica: techo de velocidad,  (sin PID)
   │     mapa de rozamiento a lo largo del riel
   │
Nivel 3  Lazo de velocidad solo                                   (PID de velocidad)
   │
Nivel 4  Lazo de posición: quieto, escalones, perfiles            (los dos PID)
   │
Nivel 5  Movimientos largos, repetibilidad, resistencia  →  interferómetro
```

### 1.1 Reglas para todas las pruebas

1. **Una variable por vez.** Cada corrida cambia un solo parámetro respecto de la anterior.
2. **Cada corrida registra su configuración**: tensión de la fuente, `M906`, `M57`, PID, si el husillo está acoplado, versión del firmware y fecha de la última `M56`. El script lo pide o lo lee de la placa y lo escribe en el encabezado del resultado.
3. **Ningún corte sin registro.** Ante cualquier corte (`fault=1`), el script guarda automáticamente `M119`, `M122` y `M121`, con la hora, *antes* de mandar cualquier otro comando. Hoy varios cortes quedaron sin traza y se perdió la pista.
4. **Pasa = pasa 3 de 3.** Cada prueba se repite al menos 3 veces seguidas. Una sola falla en 3 es "no pasa".
5. **Si una prueba no pasa, no se sube de nivel.** Se diagnostica con la tabla "Si falla" de esa prueba.
6. **Después de cualquier cambio mecánico** (desarmar el encoder, la rueda, el acople o el motor) se vuelve al nivel 1.
7. **Los umbrales son una primera propuesta.** Salen del modelo del motor y de lo medido el 06/10. Si la primera medición muestra que alguno está mal planteado, se corrige en este documento con fecha y motivo. No se cambia en silencio para que una prueba pase.

### 1.2 Dónde quedan los resultados

Cada prueba guarda en `mediciones/AAAA-MM-DD/<id>_<hora>/`:
- `resultado.txt`: configuración, valores medidos, umbral de cada criterio y **PASA / NO PASA** por criterio y en total.
- Los CSV crudos (trazas, muestras).
- Los gráficos, si la prueba los genera.

Al final de cada sesión se pasa a `docs/SESION_*.md` una tabla con id, fecha y resultado de cada prueba corrida.

### 1.3 Números de referencia

| Magnitud | Valor |
|---|---|
| Período del encoder (2 imanes) | 12° de motor = **0,1667 mm** = 2²¹ cuentas |
| Período eléctrico del motor | 7,2° de motor = **0,1 mm** |
| Paso completo | 1,8° = 25 µm |
| Un período de encoder corrido | la conmutación queda 240° eléctricos corrida: el motor empuja al revés |
| Frecuencia del lazo servo | ~22 kHz (`M122`, 09/10) |
| Corte por error de seguimiento | 1 mm durante 0,5 s |
| Rozamiento estimado el 06/10 | 14–23 mN·m (atraso de 10–16° eléctricos a 6 V, en las alineaciones del homing) |
| Torque disponible quieto | 82 mN·m a 10 V × 0,6; **165 mN·m a 12 V × 1,0** |

Techo de velocidad que predice el modelo del motor (R = 33 Ω, L = 37 mH, Kt = 0,46 N·m/A), con un rozamiento de 20–30 mN·m. Es el límite ideal: no incluye resonancias, error en la velocidad medida ni la aceleración.

| Configuración | Lazo abierto | Lazo cerrado |
|---|---|---|
| 10 V × 0,6 (hasta el 06/10) | 7–9 mm/s | **5,5–7 mm/s** |
| 12 V × 1,0, sin adelanto | > 25 mm/s | 12–13 mm/s |
| 12 V × 1,0, con adelanto L/R | > 25 mm/s | > 25 mm/s |

En lazo abierto el techo puede ser más alto que en lazo cerrado: ahí el atraso rotor-campo se acomoda solo y puede pasar de 90°, mientras que en lazo cerrado el firmware limita el adelanto a 81°. Por eso el nivel 2 y el nivel 3 se comparan entre sí.

---

## 2. Qué hay que implementar

| Pieza | Estado | Para qué prueba |
|---|---|---|
| `M119`, `M121` (con `P`/`R`), `M122`, `M55`, `M57`, `M906`, `M203`, `M201` | **existe** | todas |
| `tools/ajuste_lazo.py` (barridos de velocidad, corriente, adelanto y PID) | **existe** | N3, N4 |
| `tools/ida_vuelta.py`, `tools/run_gcode.py` | **existe** | N5 |
| `M115`: versión del firmware (fecha de compilación) y configuración activa | a implementar | todas (registro) |
| `M65 C<0/1> S<MHz>`: CRC on/off y velocidad del SPI en caliente; informa errores de CRC, lecturas descartadas y campo débil | a implementar | N0.4 |
| `M60 T<s>`: lecturas del encoder con el motor quieto; devuelve media, desvío, pico a pico, % de campo débil, CRC | a implementar | N1.1 |
| `M68`: vuelca la tabla del encoder (para comparar dos `M56`) | a implementar | N1.2 |
| `M61 N<vueltas> V<mm/s> A<mm/s²>`: lazo abierto N vueltas de ida y N de vuelta; registra la cuenta del encoder en cada vuelta completa del campo y el atraso rotor-campo | a implementar | N1.3, N2.2 |
| `M62 V<mm/s máx> A<mm/s²>`: rampa de velocidad en lazo abierto hasta perder sincronismo; informa la velocidad en que se perdió | a implementar | N2.1 |
| `M58`: mide el desfase de la conmutación (campo contra rotor, llegando de los dos lados) **sin corregir nada** | a implementar (la medición ya existe en `align_encoder_period_to_field()`) | N1.4, N5 |
| `M64 V<mm/s> T<s>`: modo velocidad, con el lazo de posición apagado | a implementar | N3 |
| `tools/banco.py`: conexión, registro automático de cortes, metadatos, escritura de `resultado.txt` | a implementar | todas |
| `tools/pruebas.py <id>`: corre una prueba por id (`n1.3`, `n2.1`…) y decide si pasa | a implementar | todas |

**Seguridad en lazo abierto:** `M61` y `M62` no tienen lazo de posición. Tienen que verificar con el encoder que el carro no salga de [5, 250] mm y cortar si se acerca a los topes. El script también chequea antes de mandar el comando que el recorrido pedido entre en el riel.

---

## 3. Nivel 0: banco

Objetivo: descartar problemas de hardware que no tienen nada que ver con el firmware. Son pruebas de una vez por montaje, y se repiten después de cada cambio de cableado.

### N0.1 Bobinas del motor

- **Desarrollo:** motor desconectado del driver. Medir con el multímetro la resistencia de las **dos** bobinas (A+/A− y B+/B−). Comprobar también que no haya continuidad entre bobinas ni entre una bobina y la carcasa.
- **Pasa si:**
  - Las dos bobinas dan 30–36 Ω (nominal 30 Ω más los cables).
  - Difieren entre sí menos del 5 %.
  - No hay continuidad entre bobinas ni a la carcasa.
- **Si falla:**
  - Bobinas distintas: falso contacto en el conector o en la protoboard (el torque varía con la posición).
  - Resistencia muy alta: cable cortado.

### N0.2 Fuente y driver bajo carga

- **Desarrollo:**
  1. Fuente a 12 V, límite de corriente 1 A.
  2. Con el motor en lazo cerrado y quieto (`M17`), medir con el multímetro la tensión en los pines VM y GND del TB6612, en la placa.
  3. Repetir durante un `G1` a la velocidad más alta que hoy funcione.
  4. Mirar el LED de CC de la fuente durante todo el movimiento.
- **Pasa si:**
  - VM ≥ 11,5 V quieto y en movimiento.
  - El LED de CC nunca se prende.
  - La corriente que marca la fuente es ≤ 0,5 A. Con 33 Ω por fase, el máximo teórico es 0,36 A por fase.
- **Si falla:**
  - VM cae y el CC está apagado: caída en los cables o en la protoboard. Llevar la potencia con cable directo, no por las pistas de la protoboard.
  - CC prendido: revisar el límite de la fuente. Con este motor, si aparece el CC el límite está mal puesto (verificar que esté en 1 A y no en 1 mA).

### N0.3 Marcas de referencia mecánicas

- **Desarrollo:** con fibrón indeleble, una raya continua que cruce:
  - la rueda de imanes y el eje del motor;
  - el eje del motor, el acople y el husillo;
  - la base del sensor MT6835 y su soporte.

  Fotografiarlas. Se revisan al final de cada sesión y siempre que una prueba de nivel 1 o 5 dé deriva.
- **Pasa si:** ninguna raya se separa.
- **Si falla:** la pieza patina. Fijarla (prisionero sobre un plano del eje, o un adhesivo para ejes) y volver al nivel 1. **Esto solo puede explicar el "hay que recalibrar con `M56`".**

### N0.4 Integridad del SPI y del CRC

El 06/10 se activó el CRC y después quedó apagado sin querer. El 09/10 se volvió a activar (`ENABLE_ENCODER_CRC = true`). Esta prueba confirma que el CRC del chip anda en todas las condiciones, porque sin él el filtro de lecturas malas solo detecta los saltos imposibles.

- **Desarrollo:**
  1. `M65 C1 S8`: CRC activo, SPI a 8 MHz.
  2. Leer contadores con el motor sin corriente durante 60 s (~1,3 millones de lecturas).
  3. Repetir con el motor en lazo cerrado y quieto, y con el motor girando en lazo abierto (`M61`, más abajo). Con corriente hay ruido del PWM del driver, que comparte la protoboard con el SPI.
  4. Repetir a 4 MHz y a 2 MHz.
- **Pasa si:**
  - 0 errores de CRC en todas las condiciones a la velocidad elegida.
  - Esa velocidad mantiene el lazo a ≥ 15 kHz en `M122`.
- **Si falla:**
  - 100 % de errores siempre: el chip devuelve un CRC que no coincide con nuestro cálculo (la nota del código original advierte que pasa con algunos chips). Revisar la implementación del CRC contra la hoja de datos. Mientras tanto se trabaja sin CRC y se documenta.
  - Errores solo con el motor energizado: ruido acoplado. Bajar el SPI, alejar los cables del encoder de los del motor, trenzar el SPI con GND, o pasar el encoder a cable corto fuera de la protoboard. Repetir hasta 0.
  - Errores también sin corriente: falso contacto en el SPI.

**Decisión al terminar N0.4:** se fija la velocidad de SPI más alta con 0 errores en las tres condiciones, con el CRC activo, para todo lo que sigue.

---

## 4. Nivel 1: encoder (sin PID)

Objetivo: comprobar que el encoder mide bien y **nunca pierde ni gana períodos**. Si el encoder cuenta mal, nada de lo que está arriba puede funcionar.

### N1.1 Ruido estático

- **Desarrollo:** `M18` (motor sin corriente), carro quieto. `M60 T10`: 10 s de lecturas. Repetir en 5 posiciones del riel (moviendo a mano o con `G1` y después `M18`).
- **Pasa si:**
  - Desvío < 0,001 período (2100 cuentas, ≈ 0,17 µm de carro).
  - 0 % de lecturas con campo débil.
  - 0 lecturas descartadas.
- **Si falla:**
  - Ruido alto en algunas posiciones: entrehierro desparejo, rueda excéntrica o imanes despegados en esa zona.
  - Ruido alto en todas: entrehierro grande o interferencia eléctrica (ver N0.4).

### N1.2 Calibración y su repetibilidad

- **Desarrollo:**
  1. `M56`, y anotar del log `rms_error`, `atraso del rotor`, `periodos por vuelta`, `desfase en la costura`, CRC y campo débil.
  2. `M68` para volcar la tabla.
  3. Repetir `M56` y `M68` sin tocar nada.
  4. El script compara las dos tablas.
- **Pasa si:**
  - `rms_error` < 0,2° de motor. El firmware aborta pasado 0,5°; 0,2° deja margen.
  - Períodos por vuelta = 30,00 ± 0,05.
  - Desfase en la costura < 1°.
  - Diferencia RMS entre las dos tablas < 0,1° de motor; máxima < 0,3°.
  - 0 errores de CRC y < 1 % de campo débil.
- **Si falla:**
  - Tablas distintas entre dos `M56`: el atraso del rotor durante la calibración (en lazo abierto) cambia de una vez a otra. Probar con más corriente o una calibración más lenta.
  - `rms_error` alto: entrehierro o rueda (N1.1).
  - Períodos ≠ 30: el encoder perdió el seguimiento durante la calibración (N0.4).

### N1.3 Conteo de períodos en lazo abierto (la prueba central del problema 2)

Es la prueba que más información da por menos esfuerzo. Mueve el motor sin ningún PID y compara el encoder contra el campo, que en lazo abierto y a baja velocidad es la referencia.

- **Desarrollo:**
  1. Carro en X ≈ 20 mm.
  2. `M61 N40 V2.5 A5`: 40 vueltas de ida (200 mm) a 2,5 mm/s y 40 de vuelta.
  3. Antes de la lectura inicial y de la final, el campo se acerca siempre **desde el mismo lado** (un período eléctrico de ida y vuelta). Así el atraso por rozamiento es el mismo en las dos lecturas.
  4. En cada vuelta completa del campo, el firmware anota la cuenta del encoder.
  5. Se repite 5 veces seguidas sin recalibrar.
- **Mediciones:**
  - Cuenta del encoder por vuelta, en períodos (tiene que ser 30).
  - **Cierre**: diferencia entre la cuenta final y la inicial, separada en parte entera (k períodos) y fracción.
  - Atraso rotor-campo en cada vuelta.
  - Descartadas y CRC.
- **Pasa si:**
  - Cada vuelta da 30,000 ± 0,02 períodos (± 0,24° de motor, lo que puede variar el atraso por rozamiento).
  - Cierre con **k = 0**, fracción < 0,02 período (3 µm).
  - El cierre no crece de una repetición a la siguiente (sin tendencia en las 5).
  - 0 descartadas, 0 errores de CRC.
- **Si falla, el valor del cierre dice qué pasó:**

| Cierre | Causa | Qué hacer |
|---|---|---|
| Un número **entero** de períodos (k ≠ 0, fracción ≈ 0) | Error de desenrollado: el encoder sumó o restó un período por lecturas malas | N0.4: CRC, velocidad del SPI, cables. Ver en qué vuelta pasó |
| Múltiplo de **0,6 período** (7,2° de motor) | El motor perdió pasos en lazo abierto (el rotor se atrasó un período eléctrico entero) | No es el encoder: velocidad demasiado alta para lazo abierto. Repetir más lento. Si pasa siempre en la misma X, es una zona dura (N2.2) |
| Una **fracción** que crece con las repeticiones | La rueda patina sobre el eje o el sensor se mueve | N0.3 (marcas de fibrón). Fijar la pieza |
| Fracción chica que no crece pero pasa el umbral | Atraso por rozamiento distinto al ir y al volver | Revisar que el acercamiento sea desde el mismo lado; si lo es, subir el umbral con justificación |

**Variante:** repetir a 1, 5 y 10 mm/s, siempre que el lazo abierto no pierda pasos (ver N2.1). Si el conteo falla solo a cierta velocidad, el problema depende de la frecuencia de lectura o del ruido del PWM a esa velocidad.

### N1.4 Conmutación a lo largo del riel

La tabla del encoder se mide en una vuelta y se repite periódicamente en todo el recorrido (~50 vueltas). Esta prueba verifica que esa suposición se cumple en todo el riel.

- **Desarrollo:**
  1. Recién hecha la `M56`, llevar el carro a 10 posiciones repartidas entre 10 y 240 mm.
  2. En cada una, `M58`: mide el desfase campo-rotor llegando de los dos lados y lo promedia. No corrige nada.
- **Pasa si:**
  - |desfase| < 15° eléctricos en las 10 posiciones.
  - La diferencia entre la posición con mayor desfase y la de menor es < 10°.
- **Si falla:**
  - El desfase crece con X: la tabla tiene un error de escala acumulativo (la cuenta por vuelta no es exactamente 30 períodos, o falla el empalme de la costura).
  - El desfase salta en una zona: el encoder perdió la cuenta al llegar ahí (N1.3).

`M58` también se usa en el nivel 5 como **"control de salud" de la conmutación**, antes y después de cada prueba larga.

---

## 5. Nivel 2: motor, driver, fuente y mecánica (sin PID)

Objetivo: conocer el **techo real** de lo que el motor puede hacer con este driver y esta fuente, y cuánto se lleva el husillo. Sin esto no se puede saber si un corte es un problema de control o de física.

### N2.1 Techo de velocidad en lazo abierto

- **Desarrollo:**
  1. `M62 V25 A5`: el campo acelera a 5 mm/s² desde X ≈ 20 mm.
  2. El firmware compara la velocidad del encoder con la del campo. Cuando el rotor se atrasa más de un período eléctrico, frena el campo e informa a qué velocidad perdió el sincronismo.
  3. Se corre en estas configuraciones, una variable por vez:
     - 10 V × 0,6 (referencia de lo de antes);
     - 12 V × 0,6;
     - 12 V × 1,0.
  4. En las dos direcciones, 3 veces cada una.
- **Pasa si:**
  - El techo con 12 V × 1,0 es ≥ 8 mm/s.
  - El techo es ≥ 2 veces la velocidad de trabajo que se pretende para el interferómetro.
  - Las 3 repeticiones dan techos a menos del 15 % entre sí.
- **Además (diagnóstico, no criterio):** comparar contra la tabla del punto 1.3.
  - Techo muy por debajo del modelo (menos de la mitad): fuente que cae (N0.2), resonancia, o rozamiento mayor que el estimado (N2.2).
  - Pérdida de sincronismo en una banda de velocidades, pero que a mayor velocidad no se pierde: **resonancia de media banda** del paso a paso, típica entre 1 y 3 vueltas/s (5–15 mm/s), no falta de torque. En lazo cerrado suele desaparecer.
- **Si falla:** con poco techo no hay control que lo arregle. Opciones, de la más barata a la más cara: menos aceleración, más tensión (hasta 13,5 V, el límite del TB6612), u otro driver.

**Opcional, con el husillo desacoplado:** el mismo ensayo. La diferencia entre acoplado y desacoplado es lo que aporta la mecánica. Con un acople rígido, desacoplar y volver a acoplar puede desalinear. Hacerlo solo si N2.1 o N2.2 dan resultados raros, y después repetir N2.2.

### N2.2 Mapa de rozamiento a lo largo del riel

Responde si "se queda a la mitad en `G1 X200`" es mecánico.

- **Desarrollo:**
  1. `M61` a velocidad baja (1 mm/s, donde la fuerza contraelectromotriz no pesa) de 10 a 240 mm y de vuelta.
  2. En cada punto se registra el atraso rotor-campo, θ(x), en grados eléctricos, ida y vuelta.
  3. El script calcula:
     - **Rozamiento**: T_roz(x) = T₀ · sen((θ_ida(x) − θ_vuelta(x)) / 2), con T₀ = el torque quieto de la configuración (165 mN·m a 12 V × 1,0).
     - **Desfase medio**: (θ_ida + θ_vuelta) / 2. Es error de la tabla más el torque de retención del motor (*detent*).
     - El **espectro** del rozamiento contra el ángulo del motor: una vez por vuelta (acople rígido desalineado o husillo torcido), 200 por vuelta (detent), etc.
- **Pasa si:**
  - Rozamiento medio ≤ 25 mN·m (< 15 % del torque quieto).
  - Ninguna zona de más de 5 mm supera 1,5 veces el rozamiento medio.
  - La componente de una vez por vuelta es < 30 % del medio.
- **Si falla:**
  - Zona dura localizada: revisar el riel y la tuerca del husillo en esa zona; anotar la X.
  - Componente de una vez por vuelta grande: **acople rígido desalineado**. Un acople rígido no tolera desalineación y la convierte en rozamiento periódico. Realinear o pasar a un acople de fuelle.
  - Rozamiento alto en todo el riel: precarga del husillo o de los rodamientos de soporte.

### N2.3 Temperatura

- **Desarrollo:** 20 minutos con 12 V × 1,0, alternando quieto en lazo cerrado y movimientos (`ida_vuelta.py 20 30`). Medir la temperatura de la carcasa del motor y del TB6612 (termómetro o termocupla; si no hay, al tacto) a los 0, 10 y 20 min.
- **Pasa si:**
  - Motor < 70 °C de carcasa.
  - TB6612 tibio, < 60 °C.
  - El techo de N2.1 medido al final no cae más de un 10 %.
- **Si falla:** la resistencia del bobinado sube con la temperatura y la corriente baja. Bajar `M906` en reposo o mejorar la disipación.

---

## 6. Nivel 3: lazo de velocidad solo

Objetivo: que el lazo interno siga una velocidad pedida sin saturar y con el rizado mínimo. Ajustar el lazo de posición sobre un lazo de velocidad que satura no converge a nada.

### N3.1 Techo en lazo cerrado

- **Desarrollo:** `M64 V<v> T3` (velocidad constante 3 s, lazo de posición apagado) para v = 0,5, 1, 2, 3, 5, 7,5, 10, 12,5, 15 mm/s, ida y vuelta, desde X ≈ 20. Se registra con `M121`.
- **Mediciones:**
  - Error medio de velocidad.
  - Rizado (desvío de la velocidad).
  - % del tiempo con la salida saturada (|out| > 1,3).
  - `out` medio, que indica qué fracción del torque se lleva el rozamiento.
- **Techo en lazo cerrado:** la mayor v con error medio < 2 % y saturación < 5 %.
- **Pasa si:**
  - Techo en lazo cerrado ≥ 70 % del techo en lazo abierto de N2.1.
  - Techo en lazo cerrado ≥ 1,5 veces la velocidad de trabajo.
- **Si falla:**
  - Techo en lazo cerrado mucho menor que en lazo abierto: es el control, no el motor. Revisar el adelanto (N3.2), la velocidad medida (zonas de la rueda) y el límite de 81° de la salida.

### N3.2 Adelanto por inductancia

- **Desarrollo:** N3.1 a la velocidad del techo y a 1,2 veces el techo, con `M57 T0`, `T0.56`, `T1.12` y `T1.7`. Con la herramienta que ya existe: `ajuste_lazo.py adelanto 0 0.56 1.12 1.7 --velocidad <v>`.
- **Pasa si:**
  - Con el valor elegido (se espera ~1,12), el techo sube respecto de `T0`.
  - El rizado a 0,5 y 2 mm/s no empeora más de un 10 %.
- **Si falla:**
  - Con adelanto anda peor que sin él: el signo está invertido, o la velocidad medida es ruidosa y el adelanto mete ruido en el campo. Revisar el signo; si es el ruido, subir el filtro de velocidad.

### N3.3 Escalones de velocidad (ajuste del PID de velocidad)

- **Desarrollo:** escalones de 1 → 2 → 1 mm/s y de 2 → 5 → 2 mm/s. Barrer `vel_kp` (0,1 / 0,2 / 0,4), `vel_ki` (45 / 90 / 180) y el filtro `vel_tc` (2 / 4 / 8 ms).
- **Pasa si:**
  - Sobrepico < 25 %.
  - Se asienta (dentro del 5 %) en < 50 ms.
  - Sin oscilación sostenida (ningún pico en el espectro de la velocidad que no esté en la consigna).
  - Sin saturar.
- **Elección:** la combinación que pasa con menor rizado a velocidad constante. Se fija en `hw_config.h`.

### N3.4 Rizado a velocidades de barrido (para el interferómetro)

- **Desarrollo:** `M64` a 0,1, 0,5, 1 y 2 mm/s durante 10 s cada una. Espectro de la velocidad medida, identificando los picos por su frecuencia:

| Frecuencia del pico | Origen probable |
|---|---|
| 1 por vuelta | Acople, husillo, rueda excéntrica |
| 30 por vuelta | Error residual de la tabla del encoder (períodos de la rueda) |
| 50 por vuelta | Desfase de la conmutación, o corriente con continua en una fase |
| 100 por vuelta | Bobinas desparejas (N0.1) |
| 200 por vuelta | Torque de retención del motor (*detent*), uno por paso completo |

- **Pasa si (provisorio):** rizado RMS < 2 % a 2 mm/s y < 5 % a 0,1 mm/s.
- **Advertencia:** esta velocidad sale del mismo encoder, así que el rizado medido incluye los errores del encoder. El rizado real del carro lo va a medir el interferómetro. Esta prueba sirve para comparar configuraciones entre sí y para encontrar de dónde viene cada pico.

---

## 7. Nivel 4: lazo de posición

Objetivo: posicionar sin sobrepico, sin oscilar quieto y siguiendo perfiles con poco error. Recién acá se ajustan `POS_KP` y `POS_KI`. `POS_KI = 30000` viene de los brazos delta del proyecto original y es probable que sobre para este eje: el cero del PI queda en 500 rad/s.

### N4.1 Quieto

- **Desarrollo:** lazo cerrado quieto 10 s en 5 posiciones, con `M121 P5`. Después, empujar suavemente el carro con el dedo y soltar.
- **Pasa si:**
  - Desvío de la posición < 0,3 µm.
  - Pico a pico < 1 µm.
  - Sin ciclo límite (oscilación sostenida).
  - Después del empujón vuelve a ±1 µm en < 200 ms, sin oscilar.

### N4.2 Escalones

- **Desarrollo:** escalones de 10 µm, 100 µm y 1 mm, hechos con `G1` con `M201 X500` y `M203 X5`, que en distancias cortas se parece a un escalón. Ida y vuelta, 5 veces cada uno.
- **Pasa si:**
  - Sobrepico < 10 % del escalón, o < 2 µm (lo que sea mayor).
  - Asienta a ±1 µm en < 150 ms.
  - Error final (según el encoder) < 1 µm.

### N4.3 Perfiles y búsqueda de PID

- **Desarrollo:** con el lazo de velocidad fijado en N3, `ajuste_lazo.py pid --velocidad <trabajo> --distancia 30`:
  1. Barrer primero `--pos-kp 30 60 120 240`.
  2. Con el mejor, `--pos-ki 0 1000 5000 30000`.
  3. Repetir a la velocidad del techo de N3.1 × 0,7.
- **Pasa si:**
  - 0 cortes.
  - Error de seguimiento máximo < 0,1 mm (10 % del umbral de corte).
  - Saturación < 5 % del movimiento.
  - Sobrepaso < 2 µm.
  - Asienta (±2 µm) en < 0,3 s después del fin del perfil.
- **Elección:** entre las combinaciones que pasan, la de menor error RMS en movimiento que además pasa N4.1 y N4.2. Se fija en `hw_config.h`.
- **Si no pasa con ninguna:** agregar **prealimentación de velocidad**: sumar a la salida del lazo de posición la velocidad del perfil, que se conoce de antemano. Con eso el lazo de posición solo corrige el error y no tiene que generar toda la velocidad. Es un cambio chico en el firmware.

---

## 8. Nivel 5: movimientos largos y repetibilidad

Objetivo: reproducir y dar por resuelto el problema 2, y dejar el eje listo para el interferómetro.

**Antes y después de cada prueba de este nivel: `M58` en X ≈ 20.** El desfase de la conmutación no debería cambiar.

### N5.1 Recorrido largo

- **Desarrollo:** `run_gcode.py` con 10 ciclos X20 → X220 → X20:
  - a 2 mm/s;
  - a 0,7 × el techo de N3.1;
  - a 0,9 × el techo (prueba de margen).
- **Pasa si:**
  - 0 cortes en las dos primeras velocidades.
  - Error final < 2 µm según el encoder.
  - Cambio del desfase de `M58` < 10° eléctricos entre antes y después.
  - 0 descartadas.
  - A 0,9 × techo se permite que corte, pero el corte tiene que ser por error de seguimiento, **nunca** por conmutación desfasada.

### N5.2 Muchas idas y vueltas cortas (el caso que hoy se degrada)

- **Desarrollo:** `ida_vuelta.py 1 500` a 2 mm/s. Con comparador en X0 si está disponible.
- **Pasa si:**
  - 0 cortes.
  - Error del firmware en cada extremo sin tendencia: la pendiente de una recta ajustada sobre los 500 ciclos es < 0,5 µm cada 100 ciclos.
  - Cambio del desfase de `M58` < 10°.
  - Comparador: desvío < 3 µm, sin tendencia.
  - Las marcas de N0.3 siguen intactas.
- **Si falla:** según cómo falló.
  - El encoder dice que está bien pero el comparador marca deriva: deriva mecánica (rueda o sensor, N0.3).
  - `M58` cambió de a poco: conmutación que se corre, volver a N1.3 con más repeticiones.
  - Cortes sin deriva: falta de margen de torque o de control (N3/N4).

### N5.3 Resistencia

- **Desarrollo:** 1 hora de ciclos mezclados (largos y cortos, a varias velocidades) con `run_gcode.py`. `M122 R` al inicio y `M122` al final.
- **Pasa si:**
  - 0 cortes.
  - 0 reinicios del watchdog.
  - `max_loop_us` < 300.
  - Cambio del desfase de `M58` < 10°.
  - La temperatura se mantiene dentro de N2.3.

### N5.4 Listo para el interferómetro

El eje está listo cuando N5.1 a N5.3 pasan 3 de 3, con la configuración final registrada (tensión, corriente, adelanto, PID, SPI y CRC) y guardada como checkpoint. El interferómetro va a medir lo que el encoder no puede ver de sí mismo: linealidad, rizado real de velocidad y deriva térmica. Esas mediciones van a un plan aparte.

---

## 9. Qué nivel detecta cada síntoma

| Síntoma | Nivel que lo aísla | Causas probables |
|---|---|---|
| Corte por encima de 2 mm/s | N2.1 (techo), N3.1 (techo en lazo cerrado) | Falta de torque: tensión, corriente, inductancia. Rozamiento (N2.2) |
| `G1 X200` se queda a mitad | N2.2 (zona dura), N1.3 (cuenta), N3.1 | Torque al límite más una zona con más rozamiento; período perdido |
| Hay que recalibrar con `M56` | N1.3, N1.4, N0.3, `M58` en N5 | Rueda o sensor que patinan (fracción); desenrollado (entero); detector de conmutación disparando en falso |
| `M17` "a cuotas" y después nada | N5 con registro automático de cortes | Mirar el `motivo=` de cada corte: error de seguimiento contra conmutación desfasada |
| Vibración o ruido quieto | N4.1, N3.3 | PID demasiado agresivo, ruido del encoder (N1.1) |
| Rizado de velocidad | N3.4 | Ver la tabla de frecuencias de N3.4 |

**Sobre la regla "si `G28` lo arregla, es un período entero; si hace falta `M56`, es mecánico":** en este firmware no alcanza para decidir. Después de un corte por conmutación desfasada, `G28` no conserva la cuenta sino que la re-ancla con la alineación por campo, y el 06/10 esa alineación mostró ±45° de dispersión contra ±60° de margen: puede elegir mal el período. Entonces "hace falta `M56`" puede pasar aunque la causa sea un período entero. N1.3 y `M58` lo separan sin ambigüedad.

---

## 10. Orden y tiempos estimados

| Paso | Qué | Tiempo aproximado |
|---|---|---|
| 1 | Implementar `M115`, `M65`, `M60`, `M58`, `M61`, `tools/banco.py` y `tools/pruebas.py` (niveles 0 y 1) | 1 sesión |
| 2 | Correr N0.1 a N0.4 y N1.1 a N1.4 | 1 sesión |
| 3 | Implementar `M62` y el análisis de N2.2. Correr el nivel 2 | 1 sesión |
| 4 | Implementar `M64`. Correr el nivel 3 y fijar el lazo de velocidad y el adelanto | 1 sesión |
| 5 | Correr el nivel 4 y fijar el PID de posición | 1 sesión |
| 6 | Correr el nivel 5. Checkpoint de la configuración final | 1–2 sesiones |

Si un nivel no pasa, el tiempo de diagnóstico y corrección se suma antes de seguir.

## 11. Registro de resultados (planilla para la sesión)

| Id | Fecha | Configuración (V, F, T, PID, SPI/CRC, acoplado) | Resultado | Valores clave | Archivo |
|---|---|---|---|---|---|
| N0.1 | | | | | |
| N0.2 | | | | | |
| … | | | | | |
