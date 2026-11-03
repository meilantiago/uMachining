# μMachining

[English](README.md) · **Español**

Firmware de lazo cerrado para un eje lineal de micromaquinado, sobre un RP2040.

Un motor paso a paso mueve un carro con un husillo de bolas. Un encoder magnético sobre el eje del motor cierra el lazo, y el motor se controla por campo (FOC), no por pasos. El firmware está adaptado de `MotionControllerRP`, de [MicroManipulatorStepper](https://github.com/0x23/MicroManipulatorStepper) (0x23), que controla un delta de 3 grados de libertad. Acá controla un único eje lineal de varias vueltas.

**Autores:** Francisco Maresca · Tiago Meilan

## Características

- Lazo en cascada posición → velocidad → ángulo de campo a ~16 kHz en un núcleo dedicado
- Calibración automática del encoder y de la conmutación sobre una vuelta completa del motor, con guardado en flash
- Filtrado de los armónicos propios del motor en la tabla de calibración
- Tabla anti-cogging, reaprendida automáticamente después de cada calibración
- Realimentación anticipada de velocidad desde el perfil, y velocidad desde la pendiente de la tabla de calibración
- Posición sin redondeo de float: vueltas enteras en enteros de 64 bits, posición y error en `double`
- Homing contra el tope mecánico, con verificación automática de la conmutación
- Corte de seguridad por error de seguimiento, con la traza del último corte
- Telemetría binaria de hasta 2 kHz e interfaz de PC con gráficos en vivo

## Rendimiento

Medido en nuestro banco a 2 mm/s, en todo el recorrido de 255 mm:

| | |
|---|---|
| Error de seguimiento en movimiento | ~1,0 µm RMS |
| Ruido de posición en reposo | ~1 nm (desvío estándar) |
| Paso más chico resuelto | 10 nm |
| Resolución del encoder en el carro | 0,08 nm por cuenta |

> **Resolución, no exactitud.** Todos estos valores los mide el propio encoder del motor: comparan la posición medida con la pedida. El error de paso del husillo, el juego y la elasticidad de la tuerca no se ven con este sensor. El carro todavía no se midió contra una referencia externa.

## Hardware

| | |
|---|---|
| Placa | Raspberry Pi Pico (RP2040) |
| Motor | Paso a paso de 1,8° (50 pares de polos) |
| Driver | TB6612 |
| Transmisión | Husillo de bolas de 5 mm por vuelta |
| Encoder | MT6835 (21 bits) frente a una rueda de 60 imanes alternados (30 períodos por vuelta) |
| Pines | Los mismos que la opción `SINGLE_AXIS_BOARD` del firmware original (ver [`src/hw_config.h`](src/hw_config.h)) |

## Puesta en marcha

### 1. Configurar

Todos los parámetros que dependen de la mecánica están en [`src/hw_config.h`](src/hw_config.h). Revisar como mínimo los marcados con `ADJUST`:

- `LEAD_SCREW_PITCH_MM`: avance por vuelta del motor
- `AXIS_MIN_POSITION_MM` / `AXIS_MAX_POSITION_MM`: límites blandos del recorrido
- `MOTOR_POLE_PAIRS`: 50 para motores de 1,8°, 100 para motores de 0,9°
- `ENCODER_MAGNET_COUNT`: cantidad de imanes de la rueda
- Los pines, si la placa es distinta

### 2. Compilar y cargar

Requiere [PlatformIO](https://platformio.org/). Cerrar el monitor serie antes de cargar.

```bash
pio run -t upload
```

### 3. Primer uso

Conectar por USB serie a 921 600 baudios y mandar:

```
M56 S      homing, calibración, reaprendizaje anti-cogging y guardado en flash
M17        habilitar el lazo cerrado
G1 X10     ir a X = 10 mm
M114       informar la posición
```

`M56` tarda unos minutos: busca el tope, barre el campo más de una vuelta en cada sentido, verifica la conmutación y hace dos idas y vueltas cortas para aprender la tabla anti-cogging. El carro necesita recorrido libre de 0 a 50 mm.

**Correr `M56` en cada encendido.** La calibración restaurada desde la flash puede dejar la conmutación corrida un período del encoder; `M56` la vuelve a medir.

### Herramientas de PC

```bash
pip install numpy pyserial PyQt5 pyqtgraph

# interfaz gráfica: control y telemetría en vivo
python3 tools/interfaz.py

# mandar comandos en secuencia e imprimir las respuestas
python3 tools/send_gcode.py "M56" "M17" "G1 X10" "M114" --timeout 600
```

`send_gcode.py` usa `/dev/ttyACM0` por defecto y la interfaz detecta la placa sola; en los dos casos se cambia con `--port`. La interfaz puede grabar la telemetría y los comandos en CSV, en `mediciones/AAAA-MM-DD/`.

## Comandos

| Comando | Descripción |
|---|---|
| `G0` / `G1 X<mm> [F<mm/s>]` | Mover. Se rechazan destinos fuera de los límites |
| `G90` / `G91` | Modo absoluto / relativo |
| `G28 [H]` | Homing contra el tope y verificación de la conmutación. `H`: tomar la posición actual como home (solo con el motor desacoplado) |
| `M17` / `M18` | Habilitar el lazo cerrado en la posición actual / apagar el motor. `M17` exige el eje calibrado y homeado |
| `M56 [S] [P] [H] [F<0\|1>] [A<0\|1>]` | Homing y calibración. `S`: guardar en flash · `P`: imprimir las mediciones · `H`: calibrar en el lugar · `F`: filtro de armónicos del motor · `A`: reaprendizaje anti-cogging |
| `M114` | Posición |
| `M119` | Estado completo, con el último corte por falla |
| `M203 [X<mm/s>]` | Velocidad máxima de `G0`/`G1` (2 por defecto, hasta 5) |
| `M55 A B C D F` | Ganancias en caliente: `POS_KP`, `POS_KI`, `VEL_KP`, `VEL_KI`, constante de tiempo del filtro de velocidad |
| `M58 V<0\|1> F<0\|1>` | Velocidad desde la tabla de calibración / realimentación anticipada |
| `M59 [S<0\|1>] [G<ganancia>] [C] [P] [W] [L]` | Anti-cogging: activar, ganancia, borrar, imprimir, guardar en / cargar de la flash |
| `M120 S<0\|1> [P<n>]` | Telemetría binaria, un punto cada `n` ticks del timer de 2 kHz |
| `M121` | Traza grabada antes del último corte por falla |
| `M57 X<mm> [F<mm/s>] [P<n>]` | Diagnóstico de conmutación: barrido a lazo abierto que imprime el desfase |

El formato de la telemetría y la arquitectura están en el [README en inglés](README.md#telemetry-format).

## Limitaciones conocidas

- **Stick-slip después de pasos chicos.** Después de un paso de unas decenas de nm el carro puede quedarse quieto ~2 s, saltar 40–80 nm cuando se rompe la fricción estática, y asentarse en ~1 nm en ~8 s.
- **Velocidad.** Probado a 2 mm/s. Cerca de 4 mm/s el motor se queda sin margen de torque con `MOTOR_MAX_CURRENT_FACTOR = 0,6`.
- **Sentido del homing.** Solo está implementado el homing contra el tope negativo.
- **Placa.** Probado solo en el RP2040.

## Licencia

MIT, igual que el proyecto original. Ver [LICENSE](LICENSE).

Gracias a 0x23 por publicar MicroManipulatorStepper: su método de calibración y el encoder con engranaje magnético hicieron posible este proyecto.
