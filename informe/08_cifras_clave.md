# 8. Cifras clave

Todas las cifras que probablemente aparezcan en el resumen y en el cuerpo del informe, con su fuente y cómo leerlas. Longitudes de carro con husillo de 5 mm.

## Diseño y sensor

| Cifra | Valor | Fuente | Nota |
|---|---|---|---|
| Resolución del encoder en el carro | 0,0795 nm por cuenta | 5 mm / (30 · 2²¹) | Resolución nominal, no ruido |
| Períodos de encoder por vuelta | 30,00 (barridos); 30,013 (calibración 06/11) | `encoder_sweep` 03/11; `m56.log` 06/11 | Rueda de 60 imanes |
| Óptimo simulado del sensor (Ø3 × 5 N35) | 1,55 mm más allá de la cara, a 1,59 mm del eje del imán | README `origin/master`; cuaderno 18/8 | |
| CV de dθ/dx en el óptimo | 0,17 % | idem | |
| Campo en el óptimo | 58 mT (N35); 70 mT (N52) | cuaderno 18/8 | El chip pide ≥ 30 mT |
| No linealidad del encoder sin corregir | 4,4–5,2 µm RMS; 15–18 µm máx. | `encoder_sweep` 29/09, 03/11 (×2) | Contra el campo comandado: sobreestima |
| Ruido del encoder, motor apagado | 0,6–0,9 nm | `mediciones/2026-11-06/RESULTADOS.md` | |

## Calibración (versión final, 06/11)

| Cifra | Valor | Fuente |
|---|---|---|
| RMS del ajuste de la tabla | 0,035° de motor = 0,48 µm | `mediciones/2026-11-06/m56.log` |
| Atraso del rotor en la calibración | 0,36° de motor = 5,0 µm | idem |
| Huella del paso del motor sacada | 1,20 µm (armónico 200/vuelta) | idem |
| Desfase de conmutación | −5,2° eléctricos | idem |
| Error de conmutación a lo largo de la vuelta (después de la corrección del 13/10) | entre −4° y +4° eléctricos (antes: rampa hasta −36°) | cuaderno 13/10 |
| Tabla de flash vencida contra calibración nueva (`M57`, un sentido) | +73 ± 18° contra +30,2 ± 3,7° | `RESULTADOS.md` 06/11 |

## Seguimiento en movimiento (2 mm/s)

| Cifra | Valor | Fuente | Nota |
|---|---|---|---|
| Error inicial (sin B1/A1), carril completo | 14,1 µm RMS; atraso 13,7 µm; ±5,2 µm por imán; máx. 25 µm | cuaderno 16/10 | |
| Atraso predicho por el límite de la integral | 12,6 µm (medido 13,25 µm) | ec. 1.7 | |
| Con B1 + A1 | 1,2 µm RMS; máx. 4,0 / 5,9 µm; saturadas 12 217 → 3 | cuaderno 16/10 | |
| Con filtro + anti-cogging, carril completo | 1,0 / 1,0 µm RMS; máx. 6,7 µm | CAMBIOS_CALIBRACION_2026-10-20 | ganancias 0,2 / 90 |
| Componente del paso con anti-cogging | 1,46 → 0,06 µm | idem | 0–50 mm |
| **Versión final, 0 → 50 → 0 mm** | **0,9 / 1,0 µm RMS; máx. 3,0 / 3,5 µm; salida máx. 46°** | `lazo_cerrado_20261106_093001` | |
| Atraso aparente por la telemetría | ±0,7 µm | REPORTE 16/10 | Artefacto, no del carro |

## Reposo y resolución

| Cifra | Valor | Fuente | Nota |
|---|---|---|---|
| Ciclo límite con `POS_KI` heredado (30 000) | 32 µm pp, 40–50 Hz | cuaderno 13/10 | |
| Caza con `POS_KI` 1500 | ±0,17 µm, 2,7 Hz | ELECCION_POS_KI | |
| Umbral de caza | `POS_KI` ≈ 200–500 | idem | Depende de la fricción |
| Oscilación tras el reacople (`VEL_KP` 0,2) | 27 nm de desvío, ~205–214 Hz | `RESULTADOS.md` 06/11 | |
| Umbral de `VEL_KP` | entre 0,12 y 0,16 | idem | |
| **Desvío en reposo, versión final (30 mm)** | **1,43 nm (mediana de 11), rango 0,79–2,94 nm** | `informe/scripts/resumen_escaleras.py` | |
| **Pasos de 10 nm** | **N = 55: 10,02 ± 0,41 nm (media ± desvío); rango 8,8–11,3 nm** | idem | Medidos por el mismo encoder |
| Desvío dentro de cada escalón | 1,0–2,9 nm (50 de 55) | idem | 2 ráfagas de ~25 Hz sin explicar |
| Piso del `float` antes del 23/10 | 3 nm a 30 mm; 24 nm a 204–255 mm | CAMBIOS_2026-10-23 | |
| Reposo a 200 mm (23/10) | 0,8 nm | idem | ganancias 0,2 / 90 |
| Stick-slip tras pasos chicos (23/10) | saltos de 40–80 nm, ~2 s de carga, ~8 s de amortiguación | idem | No verificado con las ganancias finales |

## Velocidad y torque

| Cifra | Valor | Fuente |
|---|---|---|
| Velocidad de trabajo validada | 2 mm/s | todas |
| Con carro, 3 / 4 / 5 mm/s (ganancias 0,2/90) | 2,1–3,5 / 4,4–4,6 / 3,4–3,6 µm RMS; salida > 70° el 1–9 / 33 / 40 % del tiempo | `*_acoplado_v*_resumen.txt` 03/11 |
| Fricción contra torque disponible (factor 0,6) | > 60 % | cuaderno 13/10 |
| Motor solo, lazo abierto | pérdida de paso a 33–53 mm/s según la aceleración | commit bc6dc06 (`prueba/sin-acople`) |
| Motor solo, lazo cerrado | 10 mm/s: 5,3 µm RMS | commit 73ac7f4 |
| Frecuencia del lazo servo | ~16 kHz (15,9 vueltas/ms) | Firmware del 06/11 (código del lazo sin cambios desde la medición del 23/10, `CAMBIOS_2026-10-23`) |
| Frecuencia del perfil / telemetría | 2 kHz / 1 kHz | `main.cpp` |
