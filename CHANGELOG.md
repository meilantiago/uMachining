# Historial de cambios

De lo más nuevo a lo más viejo. Cada entrada dice qué cambió, con qué parámetros quedó el lazo, cómo se validó y dónde está el detalle. Las reglas para mantener este archivo están en [docs/FLUJO_DE_TRABAJO_GIT.md](docs/FLUJO_DE_TRABAJO_GIT.md).

Git empieza el 16/10/2026. Lo anterior está reconstruido de los `docs/CAMBIOS_*.md` y de las copias de `respaldos/`.

## Parámetros del lazo en cada versión

| Desde | Versión | `POS_KP` | `POS_KI` | `VEL_KP` | `VEL_KI` | Factor de corriente | Lazo de velocidad |
|---|---|---|---|---|---|---|---|
| 06/11 | `prueba/acoplado-velocidad` | 60 | 100 | 0,12 | 54 | 0,6 | V1 F1 + anti-cogging |
| 27/10 | `v2026-10-27` | 60 | 100 | 0,2 | 90 | 0,6 | V1 F1 + anti-cogging |
| 20/10 | `v2026-10-20` | 60 | 100 | 0,2 | 90 | 0,6 | V1 F1 |
| 16/10 | `v2026-10-16` | 60 | 1500 | 0,2 | 90 | 0,6 | V1 F1 |
| 13/10 | sin git | 60 | 3000 | 0,2 | 90 | 0,6 | V0 F0 |
| antes del 13/10 | sin git | 60 | 30000 | 0,2 | 90 | 0,6 | V0 F0 |

V1 = velocidad desde la pendiente de la tabla de calibración. F1 = realimentación anticipada de la velocidad del perfil (`M58 V F`).

El factor de corriente 1,0 (con `VEL_KP` 0,12 y `VEL_KI` 54) se probó el 23/10 y se descartó. Nunca entró en master.

---

## Sin versionar / en curso

- **Identificación con chirp (`M60`).** Nunca se commiteó ni se probó en el banco. Al ordenar las ramas el 27/10 se guardó afuera del repo, en `~/Desktop/respaldo_ramas_2026-10-27/` (patch sobre `cc58abf` + archivos).
- **Limpieza para publicación** (rama `limpieza/publicacion`, `de19ab6`, 20/10): se borró sin mergear el 27/10. Sacaba `docs/` y `mediciones/` del repo; se rehace cuando se decida publicar.

## 06/11 — `VEL_KP` 0,2 → 0,12 y `VEL_KI` 90 → 54 con el motor reacoplado (`prueba/acoplado-velocidad`)

- El 03/11 se reacopló el motor. Desde entonces, el reposo a 30 mm tenía 27 nm de desvío: una oscilación de ~205–214 Hz del lazo de velocidad, no del encoder. Con el motor apagado, el encoder da 0,6–0,9 nm muestra a muestra y no tiene avisos. Su configuración (BW 5, histéresis 4) no cambió y ningún firmware escribe su EEPROM.
- Barrido con `M55` en reposo: `VEL_KP` 0,12 → 2 nm; 0,16 → 5 nm; ≥0,2 → 11 nm. `VEL_KI` casi no influye.
- La tabla de calibración en flash no correspondía a la mecánica. `M57` daba +73 ± 18° contra +30 ± 3°, y un `G1 X30` cortó por falla con la salida saturada. Con `M56` volvió a +30,2 ± 3,7°: la rueda no se corrió.
- Validación, después de `M56`:
  - Escalera de 10 nm a 30 mm: reposo de 1,3–1,6 nm; saltos de +10,3, +9,8, +10,1, +9,9 y +10,1 nm.
  - 0 → 50 → 0 a 2 mm/s: RMS de 0,9–1,0 µm y máximo de 3,5 µm. El 03/11 eran 1,7 µm y 6,3 µm.
- Nuevas herramientas: `tools/escalera_nm.py`, `tools/plot_escalera.py` (gráfico para publicar) y `tools/prueba_velocidad_acoplado.py` (03/11).
- Detalle: [mediciones/2026-11-06/RESULTADOS.md](mediciones/2026-11-06/RESULTADOS.md).
- Pendiente: guardar la calibración con `M56 S`.

## v2026-10-27 — Versión que anda (20/10 → 27/10)

Primera versión estable desde `v2026-10-20`. Junta el anti-cogging, la precisión nanométrica y la interfaz gráfica. Se grabó el 27/10 desde master con el árbol limpio y anda bien en el banco.

El 27/10, antes de regrabar, un `G1 X255` saturaba la salida a 81° desde ~50 mm y cortaba por error. No quedó claro qué firmware tenía la placa en ese momento: el del worktree de precisión tenía el factor 1,0 sin commitear. Por eso desde ahora se graba solo con el árbol limpio (ver [docs/FLUJO_DE_TRABAJO_GIT.md](docs/FLUJO_DE_TRABAJO_GIT.md)).

El 27/10 se ordenó el repo: se borraron todas las ramas salvo `master` y los worktrees.

### 27/10 — README en inglés (`0510e7e`, Francisco) e interfaz con 6 decimales (`23f8294`)
- `README.en.md` y selector de idioma en `README.md`.
- `tools/interfaz.py`: `POS_DECIMALS` 5 → 6 (1 nm en destino, G1 e indicadores). Era la única diferencia que quedaba con la interfaz del worktree de precisión.

### 23/10 — Posición sin el piso del float (merge `bee4eb9` de `precision-nanometrica`)
- `2946e69`: tabla del encoder con vueltas enteras en aritmética entera; `motor_pos`, objetivo y error en double; parser con `strtod`; perfil en double. La posición pasa entre núcleos como int32 de 0,25 nm.
- **Telemetría incompatible con la anterior:** objetivo y posición en int32 de 0,25 nm, encabezado `0xAA 0x56` (antes `0x55`). Las herramientas viejas no reciben telemetría de este firmware, y al revés.
- `8c9fc0b`: desplazamiento de vueltas cacheado y constantes del período precalculadas. El lazo vuelve a ~16 vueltas/ms (la primera versión había bajado a 13).
- `343bb7b`, `e9cf63c`: interfaz con escala hasta nm, paso de orden de magnitud elegible y autoescalado que sigue al último punto.
- Ganancias sin cambios.
- Validación: escalones de posición de 3–24 nm → <1 nm. Con esa resolución aparece un ciclo límite por fricción (saltos de 40–80 nm cada ~2 s después de pasos chicos), que no lo causa este cambio.
- Detalle: [docs/CAMBIOS_2026-10-23_PRECISION.md](docs/CAMBIOS_2026-10-23_PRECISION.md).

### 20/10 — Interfaz gráfica (`c63098c`)
- `tools/interfaz.py` (PyQt5 + pyqtgraph): control remoto y telemetría en vivo a 1 kHz, grabación a CSV. Sin cambios en el firmware.
- Guía: [docs/GUIA_INTERFAZ.md](docs/GUIA_INTERFAZ.md).

### 20/10 — Calibración con filtro de armónicos y anti-cogging (merge `3ef8eda` de `prueba/calibracion-armonicos-cogging`)
- `a24bc8b`: `M56` saca de la tabla encoder→motor los armónicos 50·m por vuelta (huella del motor), salvo los que coinciden con la rueda (`M56 F0` lo apaga). Anti-cogging con tablas de adelanto por ciclo eléctrico (`M59`), aprendidas con `tools/cogging_learn.py`.
- `1f25466`: `M56` reaprende las tablas al final (2 idas y vueltas 0→50→0 a 2 mm/s, ~2,5 min; `M56 A0` lo saltea). `M203 X<mm/s>` para la velocidad máxima (tope 5). La compensación se desvanece entre 2 y 3 mm/s.
- Activo al arrancar (`COGGING_ENABLED_AT_BOOT`).
- Validación: carril completo a 2 mm/s, RMS ~1,8 → 1,0 µm. A 4 mm/s el motor se queda sin torque (salida >70° el 35–45 % del tiempo).
- Detalle: [docs/CAMBIOS_CALIBRACION_2026-10-20.md](docs/CAMBIOS_CALIBRACION_2026-10-20.md).

## v2026-10-20 — `POS_KI` = 100 (`18d8c0b`)

- `5d00b80`: `POS_KI` 1500 → 100.
- `08961c7`: ensayo de 0 a 250 con reposos de 10 s. `POS_KI` no cambia el error en movimiento; en reposo el umbral de oscilación está entre 200 y 250. Con 100 el reposo queda en 0,002 µm y se conserva la integral de posición.
- Validación en el carril completo, contra 1500: mismo RMS (1,78 contra 1,76–1,77 µm), atraso plano en todo el eje y reposo quieto.
- Detalle: [docs/ELECCION_POS_KI.md](docs/ELECCION_POS_KI.md).

## v2026-10-16 — Primera versión en git (`e864020`)

Incluye todo lo que se hizo antes de usar git:
- Velocidad desde la pendiente de la tabla de calibración y realimentación anticipada de velocidad (B1 y A1, `M58 V1 F1` por defecto). Error en crucero 13,7 ± 5,2 µm → 0,7 ± 0,2 µm, máximo 25 → 4–6 µm.
- `POS_KI` 3000 → 1500.
- `M56 S` ya no cuelga la placa al grabar en flash (lockout de Core 1 con `multicore_lockout`).
- `M120` con telemetría completa de 28 bytes. `tools/closed_loop_test.py`.
- Validación: dos idas y vueltas 0→255→0 a 2 mm/s sin cortes.
- Detalle: [docs/CAMBIOS_2026-10-16.md](docs/CAMBIOS_2026-10-16.md), [docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md](docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md).

Ramas de prueba del mismo día (borradas el 27/10; los resultados están en `mediciones/2026-10-16/`): `prueba/v0f0-ki` (barrido de `POS_KI` en reposo con el lazo viejo) y `prueba/v1f1-ki-bajo` (barrido en movimiento).

---

## Antes de git (reconstruido)

Cada etapa dejó una copia de los archivos anteriores en `respaldos/`.

### 13/10 — La vuelta hacia el home se trababa
- La tabla encoder→motor se arma plegando las muestras sobre una vuelta. El reparto del "desfase en la costura" corría la conmutación de 0 a ~35° eléctricos dentro de cada vuelta.
- `CALIBRATION_RANGE` 370 → 400°, `CALIBRATION_TRANSIENT_SKIP` = 15°.
- `POS_KI` 30000 → 3000: elimina un ciclo límite de 40–50 Hz en reposo (32 µm → ~1 µm pico a pico).
- Nuevo `M57`: mide la conmutación en cualquier tramo del recorrido.
- Detalle: [docs/CAMBIOS_2026-10-13.md](docs/CAMBIOS_2026-10-13.md), [docs/SESION_2026-10-13.md](docs/SESION_2026-10-13.md). Respaldo: `respaldos/2026-10-13_antes_de_diagnostico/`.

### 09/10 — Conmutación desfasada (`G1 X200` se clavaba al arrancar)
- `G28` verifica y corrige la conmutación; `M56` se autoverifica.
- Habilitar y apagar el motor sin que los núcleos se pisen; PID en cero con el lazo apagado.
- `M17` exige homing; `D1` rechazado; opción `H` (home sin tope).
- Detalle: [docs/CAMBIOS_2026-10-09.md](docs/CAMBIOS_2026-10-09.md).

### 02/10 — Rueda de 60 imanes y diagnóstico de cortes
- Cantidad de imanes explícita (60) y `M56` aborta si no mide 30 períodos por vuelta.
- Velocidad máxima de 2 a 1 mm/s; `M17` rearma en la posición actual; `M119` informa el último corte; la traza `M121` se congela en el corte.
- Se detectó que el husillo es de **5 mm/vuelta**, no de 2. `LEAD_SCREW_PITCH_MM` quedó en 5 antes del 13/10.
- Detalle: [docs/CAMBIOS_2026-10-02.md](docs/CAMBIOS_2026-10-02.md).

### 29/09 — Montaje real y encoder
- El encoder estaba montado girado 90° respecto del diseño. Reubicado: sin campo débil ni saltos, 29,98 ≈ 30 períodos por vuelta.
- Homing y recorrido para 260 mm, desbordes numéricos, protecciones contra la rueda floja, herramientas de caracterización.
- Detalle: [docs/SESION_2026-09-29.md](docs/SESION_2026-09-29.md).

### Antes del 29/09 — Puesta en marcha
- Pin PWM de la bobina B corregido, `%f` en los logs, la calibración aborta ante datos inválidos.
- Detalle: [docs/HISTORIAL_DE_CAMBIOS_FIRMWARE.md](docs/HISTORIAL_DE_CAMBIOS_FIRMWARE.md).
