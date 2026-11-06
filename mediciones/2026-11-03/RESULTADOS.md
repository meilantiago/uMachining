# Pruebas del 03/11/2026 (motor sin acople)

## Barrido del encoder (`encoder_sweep_20261103_153426`)

Firmware de barrido (`variantes_main/main_barrido_encoder.cpp`), factor de corriente 0,6.

- Campo débil, overspeed, subtensión y CRC: 0. Saltos: 0. Períodos por vuelta: 30,00.
- No linealidad: 9,6 / 10,0° de encoder RMS (ida / vuelta) = 0,32–0,33° de eje. El 29/09 había dado 0,35°.
- Por ventanas de 2 períodos (`_periodos_2.png`): 7–14° RMS, la peor en los imanes 4–7. La forma se repite entre vueltas y sentidos: la corrige la tabla.

## Barridos de PID con el rotor libre (`barrido_pid_*`)

Firmware `c1c5edc`, 0 → 10 → 0 mm a 2 mm/s, 3 s de reposo en cada punta.

| Configuración | RMS | Máx | Paso del motor | Reposo |
|---|---|---|---|---|
| Base `C0.2 D90` | 1,87 µm | 4,5 µm | 2,2 µm | 0,11 µm |
| **`C0.3 D20`** | **0,92 µm** | **2,2 µm** | 0,63 µm | 0,11 µm |

- `VEL_KI` 90 es demasiado alto sin la inercia del carro: bajarlo mejora todo (45, 30, 20). Con 10 vuelve a subir el máximo.
- Con `VEL_KI` bajo se puede subir `VEL_KP`: 0,35 da casi lo mismo, 0,4 oscila, 0,5 es inestable.
- Con la base, `VEL_KP` 0,1 y 0,3 oscilan en movimiento (salida de +81 a −68°).
- Queda un atraso constante de ~0,75 µm en todas (realimentación anticipada) y el paso del motor (anti-cogging apagado en esta rama).
- Estas ganancias valen solo para el rotor libre. Con el carro, las de `master`.

## Fijación de la rueda (`prueba_rueda_*.log`, `tools/prueba_rueda.py`)

Rampas a lazo abierto con `M63` (10–40 mm/s, 300 y 1000 mm/s², los dos sentidos) y `G28` después de cada una para medir el desfase de conmutación. La rueda se había ajustado antes de la prueba.

- Factor 0,6 (`164245`): desfases entre −1,2 y +14,5° el, sin acumularse.
- Factor 1,0, fuente a 2 A, 16 rampas, 12 con pérdida de paso (`164831`): entre −8 y +14° el, desvío 6,8°, sin derivar.
- `G28` seis veces sin mover: 1,0° todas. `G28` en 9 posiciones con movimientos suaves a 1 mm/s (`posiciones`): −6,8 a +11,4°, desvío 6,5°. La dispersión depende de la posición en la vuelta, no de la rueda.
- El 30/10 los corrimientos fueron de 35–42° el y se mantenían. **Con la rueda ajustada no se corre.**
- `G28` encuentra a veces el contador corrido un período (~±120° crudo) y lo corrige solo: es la ambigüedad del cero, no la rueda.

## Encoder sin respuesta (15:45 aprox.)

`M56` falló con "0 períodos por vuelta". Con el firmware de barrido el chip devolvió 0 en todas las muestras, sin bits de estado: MISO fijo en 0. Se resolvió revisando el cableado del encoder.
