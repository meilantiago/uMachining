# POS_KI bajo con el lazo actual (V1 F1), en movimiento

Rama `prueba/v1f1-ki-bajo`. Firmware estable de `master` (v2026-10-16), con las ganancias cambiadas en caliente
con `M55` (`tools/pid_sweep.py`). Todo lo demás, como en `hw_config.h`: `POS_KP 60`, `VEL_KP 0.2`, `VEL_KI 90`,
`VEL_LOWPASS_TC 0.004`. Una sola calibración (`M56`, RMS 0,084°, conmutación −13,5° junto al home) para todas las
configuraciones. 2 mm/s, telemetría a 1 kHz.

- `barrido_pid_140929/`: eje completo, 0 → 255 → 0 mm. `POS_KI` = 1500 (base), 0, 250, 500. El barrido se cortó
  a pedido antes de 750: `ki500` quedó completa.
- `barrido_pid_142713/`: 0 → 20 → 0 mm. `POS_KI` = 1500 (base), 750, 1500 (base al final).

| POS_KI | Recorrido | RMS | Máx \|error\| | Atraso constante | Por paso (25 µm) | Resto (desvío) | Reposo en las puntas (desvío) |
|---|---|---|---|---|---|---|---|
| 1500 (base) | 255 mm | 1,78 µm | 7,5 µm | 0,69 µm | 1,73 µm | 1,02 µm | 0,45 µm |
| 0 | 255 mm | 1,83 µm | 7,8 µm | 0,79 µm | 1,76 µm | 1,01 µm | 0,14 µm |
| 250 | 255 mm | 1,78 µm | 7,4 µm | 0,68 µm | 1,75 µm | 1,00 µm | 0,13 µm |
| 500 | 255 mm | 1,78 µm | 7,4 µm | 0,68 µm | 1,77 µm | 0,98 µm | 0,15 µm |
| 1500 (base) | 20 mm | 1,77 µm | 7,9 µm | 0,68 µm | 1,76 µm | 0,98 µm | 0,26 µm |
| 750 | 20 mm | 1,77 µm | 7,4 µm | 0,69 µm | 1,76 µm | 0,97 µm | 0,22 µm |
| 1500 (base final) | 20 mm | 1,77 µm | 8,0 µm | 0,68 µm | 1,73 µm | 0,99 µm | 0,30 µm |

Gráficos, en orden de lectura:
- `resumen_simple.png`: el error en movimiento (RMS y máximo) y quieto (desvío) contra `POS_KI`.
- `trazas.png`: 1 s de error en movimiento y 1 s quieto para cada `POS_KI`, con la misma escala.
- `comparacion_eje.png`, `comparacion_metricas.png`, `comparacion_plegado.png`: el detalle (a lo largo del eje, por componente, plegado).

"Referencia, al inicio / al final" es `POS_KI = 1500` (el valor de `hw_config.h`), medido al principio y al final
del barrido para comprobar que nada cambió durante la prueba.

## Conclusiones

- **En movimiento, `POS_KI` entre 0 y 1500 no cambia el error.** RMS de 1,77–1,83 µm, máximo de 7,4–8,0 µm, y las
  mismas componentes en todas las configuraciones. Con la realimentación anticipada, la velocidad del movimiento ya
  no la tiene que dar la integral de posición, así que la integral casi no trabaja mientras el carro se mueve.
- **Con `POS_KI = 0` queda un detalle:** el atraso medio sube apenas (0,79 contra 0,68 µm) y se corre un poco a
  partir de ~130 mm (`comparacion_eje.png`). Sin integral, nada corrige un desbalance lento. Es un efecto de 0,1 µm.
- **Lo que sí cambia es el reposo** en las puntas: 0,13–0,15 µm de desvío con `POS_KI` ≤ 500, contra 0,45 µm con
  1500. Coincide con los barridos en reposo (`barrido_ki_123236`, `barrido_ki_131928`, `barrido_ki_135123`).
- **El error que queda en movimiento no depende de `POS_KI`.** Domina la componente del paso del motor (25 µm), con
  ~1,75 µm, que viene de la huella del motor en la tabla de calibración
  (`docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md`, 1.6). La salida sigue tocando 81° en picos cortos; con
  esta calibración la conmutación quedó en −13,5°.
- **Recomendación:** `POS_KI` entre 250 y 500. Mejora el reposo por 3 sin empeorar el movimiento, y conserva algo de
  integral para corregir desbalances lentos y cargas sostenidas (cosa que estas pruebas, sin carga, no miden).
  Antes de bajar a 0, probar con carga.
