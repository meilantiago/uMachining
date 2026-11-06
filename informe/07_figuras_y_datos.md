# 7. Figuras y datos

Catálogo de figuras que ya existen, con su ruta y a qué sección del informe sirven, y figuras que conviene generar. Las rutas son relativas a la raíz del repositorio. Las que están en otra rama se extraen con `git show <rama>:<ruta> > archivo.png`.

Casi todas las figuras de `mediciones/` son gráficos de trabajo (fondo oscuro, muchos paneles, títulos en jerga). Para el informe conviene regenerarlas con fondo claro, un solo mensaje por figura y etiquetas en español con unidades, como en el Informe 3.

---

## 7.1 Existentes, recomendadas

| # | Para | Figura | Ruta | Qué muestra |
|---|---|---|---|---|
| 1 | Desarrollo 2.1 | Foto de la rueda con imanes y el encoder | Cuaderno, fig. 1 (p. 3) | Montaje físico |
| 2 | Introducción 1.3 | Imán diametral contra arreglo en arco | Cuaderno, p. 4 | Uso convencional contra engranaje magnético |
| 3 | Desarrollo 2.1 | Diseño de la rueda y del soporte; ensamble del eje en Onshape | Cuaderno, pp. 21–22 | CAD |
| 4 | Desarrollo 2.3 | Líneas de campo de la fila de imanes (plano XZ) | Cuaderno, fig. 3 (p. 5) y p. 9 (30 imanes) | Por qué el campo rota entre polos |
| 5 | Desarrollo 2.3 | Ángulo del campo contra X para varias alturas | Cuaderno, p. 10 | Onda cuadrada en Z = 0, rampa lineal más arriba |
| 6 | Desarrollo 2.3 | 1 − R² y \|B\| contra Z | Cuaderno, p. 13 | La ventana la fija el campo mínimo, no la linealidad |
| 7 | **Desarrollo 2.3** | **Mapa de uniformidad (CV) y de \|B\| con el óptimo** | `origin/master:docs/magnet_uniformity_map.png` (también cuaderno p. 18, N52 y N35) | **Criterio de diseño y óptimo** |
| 8 | Desarrollo 2.3 | Ángulo del campo en el óptimo | `origin/master:docs/magnet_field_angle.png` | Recta por período |
| 9 | **Resultados 3.1** | **Barrido de caracterización del encoder** | `mediciones/2026-11-03/encoder_sweep_20261103_170456.png` (acoplado) | No linealidad, sin avisos, 30 períodos |
| 10 | Resultados 3.1 | Error por sectores de 4 imanes, dos vueltas superpuestas | `mediciones/2026-11-03/encoder_sweep_20261103_170456_periodos_2.png` | Repetibilidad entre vueltas |
| 11 | Desarrollo 2.7 | Método de plegado (4 paneles) | `mediciones/2026-10-16/reporte/fig7_metodo_plegado.png` | Datos crudos, plegado sobre el imán, período sin causa, barrido de períodos |
| 12 | Resultados 3.3 | Un cuarto de segundo en crucero | `mediciones/2026-10-16/reporte/fig1_anatomia.png` | Atraso de 13 µm y ondulación por imán |
| 13 | Resultados 3.3 | La integral en su límite | `mediciones/2026-10-16/reporte/fig2_lazo_integrador.png` | Origen del atraso (ec. 1.7) |
| 14 | **Resultados 3.3** | **Presupuesto del error** | `mediciones/2026-10-16/reporte/fig3_presupuesto.png` | Componentes por longitud |
| 15 | Resultados 3.3 | Mecanismo por imán | `mediciones/2026-10-16/reporte/fig4_iman.png` | Velocidad vista contra real |
| 16 | Resultados 3.3 | Arranque, frenado y reposo | `mediciones/2026-10-16/reporte/fig5_transitorios.png` | Transitorios |
| 17 | **Resultados 3.3** | **Antes y después de B1/A1 (4 configuraciones)** | `mediciones/2026-10-16/reporte/fig6_comparacion.png` | **Experimento de intervención** |
| 18 | Resultados 3.3 | Iteraciones del aprendizaje del anti-cogging | `mediciones/2026-10-20/cogging_153911_dos_tablas/cogging_iteraciones.png` | Convergencia en 3 iteraciones |
| 19 | **Resultados 3.4** | **Elección de `POS_KI`** | `mediciones/2026-10-20/barrido_pid_144225/eleccion_pos_ki.png` | Desvío en reposo contra `POS_KI`, movimiento constante |
| 20 | Resultados 3.4 | Trazas de reposo por `POS_KI` | `mediciones/2026-10-20/barrido_pid_144225/reposo_trazas.png` | Aparición de la caza |
| 21 | Resultados 3.3 | Versión final, 0 → 50 → 0 mm | `mediciones/2026-11-06/lazo_cerrado_20261106_093001_postcal_kp012_resumen.png` (y `_tiempo`, `_plegado`) | 0,9–1,0 µm RMS |
| 22 | **Resultados 3.6** | **Escalera de 10 nm** | `mediciones/2026-11-06/escalera_094219_claro.png` (= `origin/master:docs/steps_10nm_light.png`) | **Resolución: cinco saltos de 10 nm, 1,3 nm de desvío** |
| 23 | Resultados 3.7 | Velocidad 2–5 mm/s | `mediciones/2026-11-03/lazo_cerrado_20261103_17*_acoplado_v*_resumen.png` | Pérdida de margen de torque |
| 24 | Resultados 3.7 | Lazo abierto: velocidad del rotor y atraso contra velocidad del campo | `prueba/sin-acople:mediciones/2026-10-30/lazo_abierto_argumento.png` | Techo del motor solo |
| 25 | Desarrollo 2.4 | Interfaz de PC | `docs/img/interfaz.png` | Herramienta (opcional, apéndice) |

---

## 7.2 Para generar

| # | Para | Figura | Datos | Cómo |
|---|---|---|---|---|
| A | Desarrollo 2.1 | **Esquema del montaje** (como la fig. 1 del Informe 3): fuente → TB6612 → motor → husillo → carro; rueda + MT6835 → Pico → PC | — | Dibujo; o foto del banco anotada (la del borrador de publicación quedó pendiente) |
| B | Introducción 1.5 | Diagrama de bloques del lazo en cascada con B1 y A1 | — | Dibujo a partir del de 01 §1.5 |
| C | Desarrollo 2.5 | Calibración: muestras de ida y de vuelta (cuentas contra θ comandado) y tabla ajustada en el medio | `M56 P` (capturar con `tools/capture_calibration.py --timeout 600`) | No hay datos guardados de la versión final: hay que correrlo |
| D | Resultados 3.2 | Error de conmutación a lo largo de la vuelta, antes (rampa) y después (±4°) | `mediciones/2026-10-13/diagnostico_vuelta/sweep2.log`, `sweep3.log` | Graficar desfase contra fase de la vuelta |
| E | **Resultados 3.5** | **Desvío en reposo y potencia en 150–260 Hz contra `VEL_KP`**, con un espectro de ejemplo (0,2 contra 0,12) | `mediciones/2026-11-06/reposo_ganancias/g_*.npz`, `reposo30_*.npz`, `off_P*.npz` | Espectro de la posición en reposo; incluir el motor apagado como piso |
| F | **Resultados 3.6** | **Histograma de los 55 saltos de 10 nm** | `mediciones/2026-11-06/escalera_*.csv` | `informe/scripts/resumen_escaleras.py` (extender con un histograma) |
| G | **Resultados 3.8** | **Evolución del RMS de seguimiento y del reposo contra la fecha**, anotando cada cambio | Tabla de 03 §3.8 | Escala logarítmica en y |
| H | Resultados 3.7 | RMS y % de tiempo con salida > 70° contra velocidad (2–5 mm/s) | `*_acoplado_v*_resumen.txt` | Dos ejes |

---

## 7.3 Datos crudos

| Carpeta | Contenido | Formato |
|---|---|---|
| `mediciones/2026-10-13/` | Barridos del encoder; diagnóstico de la vuelta (`M57`) | CSV, LOG, PNG |
| `mediciones/2026-10-16/` | Carril completo con las 4 combinaciones B1/A1; barridos de `POS_KI`; figuras del reporte | LOG, TXT, PNG, JSON (NPZ solo en disco) |
| `mediciones/2026-10-20/` | Elección de `POS_KI`; anti-cogging; pruebas a 4 mm/s | idem |
| `mediciones/2026-11-03/` | Reacople: velocidad 2–5 mm/s, escalera con 27 nm, barridos con rotor libre | idem |
| `mediciones/2026-11-06/` | Versión final: escaleras, `M56`, `M57`, barrido de `VEL_KP` en reposo | CSV, LOG, NPZ |
| rama `prueba/sin-acople` | 30/10 – 03/11: torque, velocidad a lazo abierto, fijación de la rueda | idem |

Los `.npz` (telemetría cruda de `closed_loop_test.py` y de los barridos) no se versionan (`.gitignore`): están solo en el disco de esta máquina. **Hacer una copia de respaldo antes de que se pierdan**, porque varias figuras para generar dependen de ellos.

El barrido del encoder del 29/09 (`encoder_sweep_20260929_182151.csv`), citado en la documentación, no está en esta copia.
