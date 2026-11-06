# 1. Introducción y fundamentos

Material para la introducción. Va de lo general (para qué sirve el eje) a los principios físicos y de control que después se usan en el desarrollo y en los resultados. Las ecuaciones están numeradas como sugerencia.

---

## 1.1 Motivación

- **Micromaquinado láser ultrarrápido.** Con pulsos de femto o picosegundos se remueve material con muy poca zona afectada por el calor, y el tamaño de las estructuras queda limitado por el foco (del orden del µm) y por la precisión con que se mueve la pieza o el haz. Hace falta un eje que posicione con resolución submicrométrica y que se mueva a velocidad constante y pareja, porque cualquier ondulación de la velocidad cambia la dosis de pulsos por unidad de longitud y queda grabada en la pieza.
- **Ejes comerciales.** Los ejes de precisión con motor lineal o piezoeléctrico y regla óptica son caros. El proyecto busca lo mismo con componentes de bajo costo: un motor paso a paso común, un husillo de bolas, un encoder magnético de ~USD 10 y un microcontrolador RP2040.
- **Punto de partida.** El firmware se adaptó de *MicroManipulatorStepper* (0x23, licencia MIT), que maneja un micromanipulador delta de 3 ejes con motores paso a paso en lazo cerrado y un encoder MT6835 frente a una rueda de imanes. Se lo llevó a **un solo eje lineal de varias vueltas** con un husillo. Eso obligó a reescribir la calibración, la representación numérica de la posición y buena parte del lazo (ver 02, sección 2.4).
- **Objetivo del trabajo** (para reformular en el informe): diseñar y construir el eje, desarrollar su control en lazo cerrado y caracterizar su desempeño (resolución, ruido en reposo, error de seguimiento en movimiento), identificando la causa de cada contribución al error.

> El título del proyecto incluye la caracterización metrológica contra un interferómetro. Eso **no se llegó a hacer**. El informe tiene que decirlo y presentar lo hecho como caracterización en el marco del encoder (ver 1.7).

---

## 1.2 Del giro del motor a la posición del carro

El motor gira el husillo y el carro avanza un paso `p` por vuelta:

```
x = p · θ_m / (2π)                                   (1.1)
```

con `p = 5 mm` (medido el 02/10; el firmware heredado suponía 2 mm). El encoder está en el eje del motor, así que mide `θ_m` y no `x`. Todo lo que pase entre el motor y el carro (error de paso del husillo, juego, elasticidad del acople, dilatación) queda fuera de la medición.

Una consecuencia favorable: todo error angular del lado del motor (ruido del encoder, cuantización del campo, residuo de la tabla, retención del motor) llega al carro multiplicado por `p/2π = 0,80 mm/rad`. En el micromanipulador original la relación es ~15 mm/rad en el punto de unión del brazo, unas 19 veces más. **Una parte importante de la resolución que se alcanza viene de la mecánica**, y conviene decirlo así en el informe (está dicho igual en el borrador de publicación).

---

## 1.3 El encoder magnético MT6835 y el engranaje magnético

### Principio de medición (del cuaderno, clase 11/8)

- El MT6835 (MagnTek) usa **magnetorresistencia anisotrópica (AMR)**: la resistencia de una película ferromagnética depende del ángulo entre la corriente que la atraviesa y la magnetización, que en saturación sigue la dirección del campo externo.
- Dos puentes de Wheatstone de AMR, girados 45° entre sí, dan señales proporcionales a sen(2φ) y cos(2φ) (o a las dos componentes del campo en el plano). Un ASIC calcula el ángulo con un arcotangente y lo entrega con **21 bits** (2²¹ = 2 097 152 cuentas por vuelta del campo) por SPI.
- Como trabaja en saturación, **mide la dirección del campo, no su intensidad**. Necesita un campo mínimo para estar saturado (30 mT según la hoja de datos). Por debajo activa un bit de *campo débil* y entrega la lectura igual. No informa cuánto vale el campo.

> Para el informe: verificar en la hoja de datos el número exacto de puentes, la orientación y el rango de campo. El cuaderno dice "dos pares de puentes desfasados 45°".

### Engranaje magnético

En el uso habitual, un imán diametral en la punta del eje da una vuelta de campo por vuelta mecánica. Acá el sensor está frente a una **rueda de N = 60 imanes cilíndricos con polaridad alternada**. Al pasar un par N–S frente al sensor, el campo da una vuelta completa. Con `N_p = N/2 = 30` pares:

```
θ_enc = N_p · θ_m        (módulo 2π)                  (1.2)
```

y la resolución en el carro pasa a ser

```
δx = p / (N_p · 2²¹) = 5 mm / (30 · 2 097 152) = 0,080 nm por cuenta     (1.3)
```

El precio es que el ángulo del campo **no es exactamente lineal** con la posición a lo largo de cada par de polos. Esa no linealidad, si se repite, se corrige con una tabla de calibración (02, sección 2.5).

### Linealidad del campo: el criterio de diseño

Para un sensor que se desplaza a lo largo de la fila de imanes, el ángulo medido es `θ(x) = arctan2(B_z, B_x)` en el plano que mide el chip. La lectura es lineal si `θ` crece a ritmo constante con `x`. Derivando:

```
dθ/dx = (B_x · dB_z/dx − B_z · dB_x/dx) / (B_x² + B_z²)          (1.4)
```

y la uniformidad se cuantifica con el coeficiente de variación a lo largo de un período:

```
CV = desvío estándar(dθ/dx) / |media(dθ/dx)|                      (1.5)
```

`CV = 0` es linealidad perfecta. La condición equivalente es que la figura de Lissajous `(B_x, B_z)` sea un círculo recorrido a velocidad angular constante. Este criterio unifica los dos que se usaron primero por separado en el cuaderno (R² del ajuste lineal del ángulo y circularidad del Lissajous), y además penaliza las zonas de campo débil, donde el denominador de (1.4) se anula y aparecen picos.

Reglas generales que salieron de la simulación (cuaderno 14/8 y 18/8):
- Frente al plano medio de los imanes (`z = 0`) el campo es puramente axial y el ángulo salta ±90°: el sensor queda ciego. Hay que estar por encima o por debajo de la cara.
- Lejos de la fila (entrehierro mayor que ~medio paso polar) los campos de imanes vecinos se mezclan, aparecen armónicos espaciales y el campo cae.
- **La linealidad depende solo de la geometría** (tamaño, paso, posición del sensor). El grado del imán (N35, N52) solo escala la intensidad. Se verificó: mismo óptimo y mismo CV para N35 y N52.

---

## 1.4 Control por campo de un motor paso a paso

### El motor paso a paso como máquina sincrónica

Un motor híbrido de 1,8° tiene `p_p = 50` pares de polos: el ciclo eléctrico es de 7,2° mecánicos (100 µm de carro) y el paso completo de 1,8° (25 µm). Con dos bobinas alimentadas con corrientes `I·cos(φ_c)` e `I·sen(φ_c)`, el estator genera un campo que apunta en el ángulo eléctrico `φ_c`. El rotor desarrolla un torque

```
τ ≈ K_t · I · sen(φ_c − p_p·θ_m)  =  K_t · I · sen(δ)              (1.6)
```

donde `δ` es el **adelanto del campo respecto del rotor** (en grados eléctricos). En el modo tradicional (lazo abierto) se gira `φ_c` y se supone que el rotor sigue; si la carga exige `δ > 90°`, el rotor pierde el paso y nadie se entera.

### Control por campo (FOC) con el encoder

En lazo cerrado se mide `θ_m`, se calcula el ángulo eléctrico del rotor y se impone `φ_c = p_p·θ_m + δ`, donde `δ` lo decide el controlador. Así el torque se controla con `δ` y el motor no puede perder el paso. En el firmware:

- `δ` es la **salida del lazo de velocidad**, limitada a **±81°** (0,45π), donde sen(δ) = 0,988: prácticamente el torque máximo.
- La amplitud de la tensión es fija (factor 0,6 de la fuente). El TB6612 es un puente H que aplica tensión, no regula corriente, así que la corriente queda determinada por `V/R` en reposo y cae con la velocidad por la inductancia (`τ_L = L/R ≈ 37 mH / 33 Ω ≈ 1,1 ms`; el campo real llega atrasado `arctan(ω_e τ_L)`).
- Con el carro quieto `δ ≈ 0`. Moviéndose a 2 mm/s, `δ ≈ 30–45°` solo para vencer la fricción.

> El cuaderno (12/9) dice que el campo se mantiene "siempre 90° adelante del rotor". Es incorrecto y se corrigió el 16/10: el adelanto es la salida del PI, ±81° como máximo, ~0° quieto.

### Datos del motor (de `docs/PLAN_DE_PRUEBAS_MODULAR.md`)

R = 33 Ω por fase, L = 37 mH, K_t = 0,46 N·m/A. Torque quieto disponible: 82 mN·m a 10 V × 0,6, 165 mN·m a 12 V × 1,0. Rozamiento estimado del eje acoplado: 14–30 mN·m. **Falta el modelo del motor y la tensión de la fuente con la que se hicieron las mediciones finales** (ver 09).

---

## 1.5 El lazo de control en cascada

```
              ┌────────────── v_perfil (realimentación anticipada, A1) ─────────────┐
x_obj ──►(+)──► PI posición (K_pp=60, K_ip=100) ──►(+)──► v_pedida ──►(+)──► PI velocidad (K_pv=0,12, K_iv=54) ──► δ (±81°)
          ▲−                                                          ▲−
          │                                                           │
  x_med ◄─ tabla de calibración ◄─ encoder          v_med ◄─ pendiente de la tabla (B1), filtro 4 ms
```

- Lazo externo (posición): convierte el error de posición en una velocidad pedida.
- Lazo interno (velocidad): convierte el error de velocidad en el adelanto `δ`, es decir, en torque.
- Corre en el núcleo 1 del RP2040, a ~16 kHz en la versión final. El perfil trapezoidal se calcula en el núcleo 0 a 2 kHz.

Relaciones útiles:

```
Error constante en crucero sin realimentación anticipada:
  e = (ω_req − ω_I,max) / K_pp                                       (1.7)
  con ω_req = 2π v / p = 2,518 rad/s a 2 mm/s y ω_I,max = π/2 (límite de la integral)
  → e = 0,0158 rad = 12,6 µm  (medido: 13,25 µm)

Escala de tiempo de la integral de posición:  T_i = K_pp / K_ip       (1.8)
  K_ip = 100 → 0,6 s;   K_ip = 1500 → 0,04 s
```

### Fricción estática y ciclo límite

Un PI contra fricción de Coulomb con fricción estática mayor que la dinámica tiene un ciclo límite (*hunting*, *stick-slip*): el error queda dentro de la franja que la fricción no deja corregir, la integral se carga, el torque vence la fricción, el carro salta, la integral se pasa y el ciclo se repite al revés. Cuanto más grande la ganancia integral, más rápido y más amplio. Es la explicación de las oscilaciones en reposo de 32 µm (POS_KI = 30 000), ±0,17 µm (1500) y de los saltos de 40–80 nm después de pasos chicos (resultados, 3.6 y 3.8). Referencia clásica: Armstrong-Hélouvry et al. (1994).

---

## 1.6 Calibración del encoder: idea

La no linealidad del ángulo del encoder es repetible: es la misma cada vez que pasa el mismo imán. Se mide una vez y se corrige con una tabla `cuentas → θ_m`. La "verdad" con la que se compara es el **ángulo de campo comandado** en lazo abierto, dividido por los 50 pares de polos. Como el rotor va atrasado respecto del campo (por fricción) y el atraso cambia de signo con el sentido, se barre en los dos sentidos y se ajusta una sola tabla por mínimos cuadrados, que queda en el promedio y cancela el atraso. Detalles en 02, sección 2.5.

Dos separaciones por frecuencia que se usan en todo el trabajo:

| Origen | Se repite por vuelta de motor | Longitud en el carro |
|---|---|---|
| Rueda de imanes | 30·m veces (período del encoder) / 60·m (imán) | 166,7 µm / 83,3 µm |
| Motor | 50·m veces (ciclo eléctrico) / 200·m (paso) | 100 µm / 25 µm |
| Husillo, acople, excentricidad | 1·m veces | 5 mm |

Coinciden solo en 150, 300, 450… por vuelta (cada 33,3 µm). Eso es lo que permite sacar de la tabla la "huella" del motor (02, 2.5.4) y separar el error en componentes (02, 2.7).

---

## 1.7 Resolución, precisión y exactitud

Vocabulario del VIM (JCGM 200), que conviene usar con cuidado en el informe:

| Término | Qué es | En este trabajo |
|---|---|---|
| Resolución | Menor cambio que el sistema puede indicar | 0,08 nm por cuenta; en la práctica, el ruido en reposo (~1,4 nm) y el menor paso que se ve (10 nm) |
| Repetibilidad | Dispersión de mediciones repetidas en las mismas condiciones | Error de seguimiento entre idas iguales: coincide en centésimas de µm |
| Veracidad / exactitud | Cercanía al valor verdadero | **No medida**: requiere referencia externa |
| Error de seguimiento | Objetivo − posición medida por el encoder | Es lo que se midió en todo el trabajo |

Para el informe: el eje cumple dos papeles. Como **instrumento** (`M114` informa la posición) su error es el error de medición del encoder más el de transmisión. Como **posicionador**, el error es ese mismo más el error de seguimiento. Lo medido es solo el último término. Con el carro quieto el error de seguimiento es de nanómetros, así que la exactitud de posicionamiento queda dominada por términos que todavía no se midieron.
