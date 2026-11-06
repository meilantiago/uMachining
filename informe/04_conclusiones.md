# 4. Conclusiones

Material para la sección de conclusiones y para el resumen. Separado en lo que se puede afirmar con los datos, lo que no, y el trabajo futuro. Las cifras están en [08_cifras_clave.md](08_cifras_clave.md).

---

## 4.1 Lo que se puede afirmar

1. **Se construyó y se puso en marcha un eje lineal de precisión de bajo costo** (motor paso a paso de 1,8°, husillo de bolas de 5 mm, guía de rodillos cruzados, 255 mm de recorrido útil) con control en lazo cerrado por campo sobre un RP2040.

2. **El diseño del encoder se guió por simulación**, y la simulación fue determinante:
   - la linealidad del ángulo del campo depende solo de la geometría (tamaño, paso y posición del sensor); el grado del imán solo fija la intensidad;
   - el límite práctico de la posición del sensor no es la linealidad sino el campo mínimo del MT6835 (30 mT);
   - el criterio unificado (CV de dθ/dx con |B| ≥ 30 mT) dio el óptimo para imanes Ø3 × 5 mm N35: 1,55 mm más allá de la cara, en el borde del imán, con 58 mT y CV 0,17 %;
   - el mismo modelo explicó la falla del primer montaje (sensor girado 90°, midiendo en el plano equivocado).

3. **El engranaje magnético funciona como se esperaba**: 30,00 períodos por vuelta, sin avisos del chip, 0,08 nm por cuenta en el carro. Su no linealidad (~5 µm RMS, ~15–18 µm máx.) es repetible y estable en semanas, y la corrige una tabla periódica de una vuelta (residuo del ajuste: 0,48 µm RMS en la versión final).

4. **El error de seguimiento a 2 mm/s bajó de 14,1 µm a 0,9–1,0 µm RMS**, y cada reducción tiene una causa identificada y verificada por intervención:
   - 13 µm de atraso constante: el límite de la integral de posición (predicho 12,6 µm, medido 13,25 µm), eliminado con realimentación anticipada de velocidad;
   - ±4,5 µm por imán: la velocidad calculada con el ángulo sin corregir, eliminada calculándola con la pendiente de la tabla;
   - la huella del motor en la calibración (paso de 25 µm): filtrada por frecuencia y compensada con anti-cogging aprendido (1,46 → 0,06 µm en esa componente).

5. **El método de análisis por plegado** separa el error en componentes por la longitud con que se repiten (imán, paso, ciclo eléctrico, vuelta). Es muy selectivo (picos de ~0,03 µm de ancho en el barrido de períodos) y reduce el ruido aleatorio a centésimas de µm. Es reutilizable para cualquier eje con elementos periódicos.

6. **En reposo, el lazo mantiene la posición medida con ~1,4 nm de desvío** (mediana de 11 corridas, rango 0,8–2,9 nm), del orden del ruido del sensor con el motor apagado (0,6–0,9 nm), **y resuelve pasos de 10 nm**: 55 saltos de 10,02 ± 0,41 nm. Para llegar a esto hubo que:
   - bajar `POS_KI` de 30 000 a 100, por el ciclo límite contra la fricción estática;
   - sacar la posición del `float` de 32 bits (piso de 3–24 nm según la posición);
   - bajar `VEL_KP` a 0,12 después del reacople del motor (oscilación de 214 Hz).

7. **Las ganancias del lazo de velocidad y la calibración dependen de la mecánica.** Después de un desmonte, la tabla de la flash dejó la conmutación corrida (+73° contra +30°) y el lazo de velocidad quedó al borde de la inestabilidad. Procedimiento: `M56` en cada arranque y revisar ganancias después de cada cambio mecánico.

## 4.2 Lo que no se puede afirmar todavía

- **La exactitud del carro.** Todas las cifras son del encoder del motor contra el objetivo. El error de paso del husillo, el juego de la tuerca, la elasticidad del acople y la dilatación térmica no se ven. Que el lazo resuelva 10 nm no implica que el carro se mueva 10 nm.
- **Si la ondulación de 25 µm y la de 83 µm eran movimiento real o error de medición.** La explicación del mecanismo es consistente, pero solo un sensor externo rápido lo confirma.
- **La reproducibilidad de la calibración** entre `M56` sucesivos (no se comparó la tabla de varias calibraciones seguidas).
- **El desempeño de la versión final en todo el recorrido y a más de 2 mm/s**: con las ganancias finales solo se midió 0–50 mm a 2 mm/s y la escalera a 30 mm.
- **La incertidumbre de las cifras**: no hay presupuesto de incertidumbre.

## 4.3 Trabajo futuro (en orden de prioridad)

1. **Referencia externa** (el objetivo original del proyecto): interferómetro, o comparador de 0,1 µm en carreras cortas. Ensayos propuestos en `docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md` §3.4, al estilo ISO 230-2:
   - E1: repetibilidad unidireccional y reversa en 3 puntos;
   - E2: mínimo desplazamiento real (pasos de 0,5–5 µm);
   - E3: error instrumental dentro de la vuelta (encoder + tabla), plegado;
   - E4: exactitud a lo largo del recorrido, 11 puntos con separación irregular, 5 ciclos bidireccionales;
   - E5: deriva térmica;
   - E6: verificación dinámica con sensor rápido (confirma 83 µm y 25 µm).
2. **Mecánica del encoder**: soporte más rígido (hoy voladizo de 56 mm con ~0,3 mm de juego) y rueda rediseñada (un alojamiento por imán, buje en D). Simulado: el error antes de la tabla bajaría de 15,5 a 3,8 µm pp y el campo mínimo subiría de 30 a 38 mT. No construido.
3. **Margen de torque**: más corriente (el TB6612 admite 1,2 A), compensación del atraso L/R de la corriente, revisar la alineación del acople (pico de fricción una vez por vuelta). Habilitaría velocidades de 5–10 mm/s con el carro.
4. **Stick-slip después de pasos chicos**: revisar con las ganancias finales; si persiste, compensación de fricción o integral con zona muerta.
5. **Compensación del husillo**: con E4, una tabla mm pedidos → mm reales en el firmware; o un encoder lineal sobre el carro (el lazo en cascada ya tiene un esqueleto, `cascade_corrector`, desactivado).
6. **Extensión a XY**.

## 4.4 Frases para el resumen (borrador)

> Se diseñó, construyó y caracterizó un eje lineal de posicionamiento para micromaquinado láser basado en un motor paso a paso de bajo costo controlado por campo en lazo cerrado. La posición se mide con un encoder magnético MT6835 de 21 bits frente a una rueda de 60 imanes alternados que multiplica por 30 su resolución, dando 0,08 nm por cuenta en el carro. La geometría de la rueda y la posición del sensor se optimizaron mediante simulaciones del campo magnético, minimizando la variación de la velocidad de rotación del campo con la condición de un campo mínimo de 30 mT. La no linealidad del encoder (~5 µm RMS) se corrigió con una tabla de calibración ajustada por mínimos cuadrados sobre una vuelta del motor. El error de seguimiento se separó en componentes mediante promedios sincrónicos sobre las longitudes características del sistema, lo que permitió identificar y eliminar sus causas: el error a 2 mm/s se redujo de 14 µm a 1 µm RMS. En reposo, la posición medida tiene un desvío de 1,4 nm y se resuelven pasos de 10 nm (10,02 ± 0,41 nm, N = 55). Estos valores caracterizan la resolución y la estabilidad del lazo en el marco del encoder; la exactitud del carro requiere una referencia externa, que queda como trabajo futuro.
