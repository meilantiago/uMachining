# Barrido de POS_KI con el lazo anterior (V0F0), en reposo en X10

Rama `prueba/v0f0-ki`: firmware estable de `master` (v2026-10-16) con `VELOCITY_FROM_LUT = false` y
`VELOCITY_FEEDFORWARD = false`, o sea el lazo de antes del 16/10. El firmware informó
`velocidad_tabla=0 realimentacion_anticipada=0` en las 5 corridas.

Para cada `POS_KI`: compilar, cargar, `M56`, `G1 X10`, y 10 s de error (objetivo − posición) a 1 kHz una
vez asentado. Herramienta: `tools/ki_sweep.py --values 0 250 500 750 1000 --positions 10`.

| POS_KI | Media | Varianza | Desvío | Pico a pico | Qué se ve |
|---|---|---|---|---|---|
| 0 | −0,002 µm | 0,0000 µm² | 0,005 µm | 0,03 µm | Quieto |
| 250 | −0,011 µm | 0,0028 µm² | 0,052 µm | 0,53 µm | Termina de asentarse (−0,5 µm al principio) y después queda quieto |
| 500 | +0,000 µm | 0,0024 µm² | 0,049 µm | 0,17 µm | Quieto 4 s; después arranca un ciclo de ±0,08 µm |
| 750 | −0,001 µm | 0,0001 µm² | 0,012 µm | 0,05 µm | Oscilación lenta y amortiguada de ±0,02 µm |
| 1000 | +0,001 µm | 0,0385 µm² | 0,196 µm | 0,48 µm | Ciclo límite completo, ±0,23 µm, ~2,4 Hz |

- **La media es cero en todos los casos.**
- **El ciclo de "caza" en reposo** (onda cuadrada, la integral contra la fricción estática) **aparece entre 500 y 1000**. Por debajo, el carro queda quieto dentro de centésimas de µm.
- **500 y 750 no siguen un orden:** con una sola corrida por valor, en ese rango que el ciclo arranque o no depende de dónde queda parado el carro. Para fijar el umbral hacen falta varias repeticiones por valor.
- **Comparado con el lazo actual** (V1F1, barrido de las 12:32 en X20/X100): con 500 había un ciclo de ±0,13 µm desde el principio, y con 1100 un desvío de 0,21 µm (acá, con 1000: 0,20 µm). En reposo los dos lazos se comportan parecido, como era de esperar: con el carro quieto la velocidad es ~0 y la anticipada no aporta nada.
- **Este ensayo es solo en reposo.** `POS_KI` bajo puede empeorar el seguimiento en movimiento (sin la anticipada, con `POS_KI = 0`, el 13/10 se midieron 33 µm de atraso a 2 mm/s) y el rechazo de cargas.
