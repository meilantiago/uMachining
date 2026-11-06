# Material para el informe técnico

Esta carpeta junta lo necesario para escribir el informe final del proyecto *Diseño, construcción y caracterización metrológica de un eje de posicionamiento de precisión para micromaquinado láser ultrarrápido* (Laboratorio 6, grupo 33: Tiago Meilán y Francisco Maresca; dirección: Gastón Corthey y Martín A. Gambarotta; UNSAM).

No es el informe: es la base para fundamentarlo. Cada cifra lleva el archivo de donde sale, para poder citarla o rehacerla.

## Archivos

| Archivo | Para qué sección del informe | Contenido |
|---|---|---|
| [01_introduccion_y_fundamentos.md](01_introduccion_y_fundamentos.md) | Introducción | Motivación, principio del encoder magnético con engranaje magnético, control por campo de un paso a paso, lazo en cascada, resolución contra exactitud, ecuaciones |
| [02_desarrollo_experimental.md](02_desarrollo_experimental.md) | Desarrollo experimental | Montaje, diseño de la rueda de imanes (simulación), firmware, calibración, protocolos de cada ensayo, método de análisis (plegado) |
| [03_resultados_y_discusion.md](03_resultados_y_discusion.md) | Resultados y discusión | Resultados en orden lógico, con tablas, fuente y discusión de cada uno |
| [04_conclusiones.md](04_conclusiones.md) | Conclusiones | Qué se puede afirmar, qué no, y trabajo futuro |
| [05_apendice.md](05_apendice.md) | Apéndice | Equivalencias de unidades, parámetros finales, comandos, telemetría, cronología de problemas resueltos |
| [06_bibliografia.md](06_bibliografia.md) | Referencias | Hojas de datos, proyecto de origen, métodos y normas |
| [07_figuras_y_datos.md](07_figuras_y_datos.md) | Todas | Catálogo de figuras que ya existen y figuras que conviene generar, con la ruta de cada una |
| [08_cifras_clave.md](08_cifras_clave.md) | Resumen y todas | Las cifras que van a aparecer en el informe, en una sola tabla, con fuente y advertencias |
| [09_inconsistencias_a_resolver.md](09_inconsistencias_a_resolver.md) | — | Valores que fija el firmware del 06/11 cuando el cuaderno dice otra cosa; lo que queda por resolver o confirmar |
| [scripts/resumen_escaleras.py](scripts/resumen_escaleras.py) | Resultados | Consolida las escaleras de 10 nm en una tabla |

## Versión que se documenta

La versión fija es la del **06/11/2026**. Si el cuaderno o un archivo de sesión dice otra cosa que el firmware de esa versión, manda el firmware (ver [09](09_inconsistencias_a_resolver.md), sección 9.1). Los parámetros del lazo son los mismos en las dos ramas de ese día:

- rama `prueba/acoplado-velocidad`, commit `9b913bc` (firmware probado en el banco). La rama `informe-tecnico` sale de ese commit, con el mismo firmware, y además junta los datos de `prueba/sin-acople` y las simulaciones y figuras de `master` (ver «Fuentes que se usaron»);
- `master` local `5e0ab3e` y `origin/master` `cd2fc77` (versión publicada: firmware limpio, mensajes en inglés, simulaciones en `simulations/magnet_wheel.ipynb`).

| Parámetro | Valor final |
|---|---|
| `POS_KP` / `POS_KI` | 60 / 100 |
| `VEL_KP` / `VEL_KI` | 0,12 / 54 |
| `VEL_LOWPASS_TC` | 4 ms |
| Factor de corriente | 0,6 |
| Velocidad desde la tabla (B1) y realimentación anticipada (A1) | activas (`M58 V1 F1`) |
| Filtro de armónicos del motor en la calibración | activo |
| Anti-cogging aprendido, reaprendido en cada `M56` | activo |
| Velocidad por defecto / aceleración | 2 mm/s / 5 mm/s² |

## Fuentes que se usaron

- Cuaderno de laboratorio (PDF, clases del 11/8 al 16/10).
- Informe 3 (difusividad térmica) como modelo de estructura y estilo.
- `docs/*.md`, `CHANGELOG.md`, `README.md` de esta rama.
- Mediciones de `mediciones/2026-09-29` a `mediciones/2026-11-06`.
- Historia de git de todas las ramas, en particular `prueba/sin-acople` (30/10 a 03/11, motor desacoplado), En la rama `informe-tecnico` se trajeron sus mediciones (`mediciones/2026-10-30/` y los archivos del 03/11 con el rotor libre), `tools/prueba_torque.py`, `prueba_velocidad.py`, `prueba_rueda.py` y `contribucion_upstream/`. Su firmware (sin homing, recorrido ±500 mm, `motor_libre`) no se trajo: sigue en `prueba/sin-acople`.
- De `master` (`cd2fc77`): `simulations/magnet_wheel.ipynb` y las figuras `docs/magnet_field_angle.png`, `docs/magnet_uniformity_map.png`, `docs/steps_10nm_{light,dark}.png`.
- `README.md` y `contribucion_upstream/SHOWCASE.en.md` (borrador de publicación, rama `prueba/sin-acople`).

## Qué dejar afuera del informe

El cuaderno y los `docs/` tienen mucho detalle de depuración que no hace al informe: pines mal asignados, desbordes de enteros, cuelgues al grabar la flash, bucles infinitos del homing, nombres de comandos de diagnóstico. Conviene mencionarlo, si acaso, en una línea ("la puesta en marcha requirió corregir varios errores del firmware heredado"). Lo que sí va: las decisiones de diseño con su fundamento y las mejoras con su medición antes y después.

## Advertencia que tiene que estar en todo el informe

**Todas las cifras de desempeño son errores de seguimiento medidos por el propio encoder del motor**: la posición que mide el encoder contra la que se le pidió. Hablan de resolución y de estabilidad del lazo, no de exactitud. El paso del husillo, el juego de la tuerca y la elasticidad del acople no los ve el encoder. El carro todavía no se midió contra una referencia externa (interferómetro). Ver [09_inconsistencias_a_resolver.md](09_inconsistencias_a_resolver.md) y [04_conclusiones.md](04_conclusiones.md).
