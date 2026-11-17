# Tests de PC del firmware

Corren en la computadora, sin la placa ni el stage:

```bash
python tests_host/run_tests.py
```

## Qué prueban

**Integración del `M60` (`test_senoide.cpp`).** Compila los archivos reales del firmware que no dependen del hardware: `command_parser.cpp`, `sine_command.cpp`, `sine_profile.cpp`, `trapezoidal_profile.cpp` y `setpoint.h`. Este último es la misma composición trapecio + senoide que usa la interrupción de 2 kHz. Una máquina de PC recibe texto por el `CommandParser` real y avanza la consigna con ticks de 500 µs ± 20 µs. Verifica:

- respuestas y errores del comando (rangos, eje sin home, motor apagado, velocidad pico, recorrido, senoide en curso);
- tiempos redondeados a ciclos enteros;
- amplitud, frecuencia y forma de la ventana de medición, de 1 a 250 Hz y con 100 nm;
- que la ventana dure exactamente N ciclos y arranque con fase 0;
- que no haya saltos, que la velocidad coincida con la derivada de la posición y que termine en desvío cero;
- la senoide sobre un `G1`, el frenado con `M60 S0`, el corte en seco y un barrido de 13 frecuencias.

**Punta a punta (`test_e2e_prueba4.py`).** El barrido generado por el código del firmware entra a la máquina simulada de la prueba 4 (`simulaciones/prueba4_simulacion.py`: lazo del firmware, mecánica con fricción, Michelson en cuadratura, relojes distintos). El análisis (`tools/prueba4_analisis.py`) tiene que encontrar los 13 tramos solo con la marca de medición y estimar el Bode con menos de 0,05 dB y 0,5° de error respecto de la verdad.

## Compilador

Usa `g++` o `clang++` si están instalados. Si no hay ninguno, usa el clang empaquetado de PyPI:

```bash
python -m pip install --user ziglang
```

## Lo que no cubren

El pegamento con el hardware no se ejecuta en estos tests:
- `SingleAxisCommandProcessor::process_sine_command`, que arma el contexto desde el eje;
- la interrupción de `main.cpp`: el pin `PIN_SINE_SYNC`, los bits de telemetría y el corte por falla.

Se verifica solo que compile para el RP2040 (`python -m platformio run`). Falta probarlo en el banco.
