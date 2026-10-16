# Firmware de eje único (X) — adaptado de MicroManipulatorStepper

Este firmware es una adaptación de `MicroManipulatorStepper/firmware/MotionControllerRP`
(0x23, MIT license) para controlar **1 solo eje lineal** (carro + husillo sobre guía de
rodillos cruzados) en vez de las 3 juntas del brazo delta original.

## Origen de cada archivo

**Copiados tal cual del repo original** (sin ninguna modificación de lógica):
- `src/hardware/MT6835_encoder.h` / `.cpp`
- `src/hardware/TB6612_motor_driver.h` / `.cpp`
- `src/servo_control/pid.h` / `.cpp`
- `src/servo_control/servo_controller.h` / `.cpp`
- `src/servo_control/homing_controller.h` / `.cpp`
- `src/servo_control/actuator_calibration.h` / `.cpp`
- `src/utilities/lookup_table.h` / `.cpp`
- `src/utilities/logging.h` / `.cpp`
- `src/utilities/math_constants.h`
- `src/utilities/fp_math.h`
- `src/command_parser/command_parser.h` / `.cpp` (se le sacó un `#include` no usado a
  `command_parser.h`, cero cambios de lógica)

**Escritos de cero para este proyecto** (no existen en el repo original):
- `src/hw_config.h` — pines de la variante de 1 eje (mismo pinout que la opción
  `SINGLE_AXIS_BOARD` ya prevista en el `hw_config.h` original) + parámetros propios
  (paso de husillo, límites de recorrido, velocidad/aceleración máximas).
- `src/motion_profile/trapezoidal_profile.h` / `.cpp` — generador de perfil de
  velocidad trapezoidal/triangular punto-a-punto. Reemplaza al `PathPlanner` +
  `MotionController` + modelo cinemático delta del original, que resolvían un
  problema mucho más complejo (3 ejes acoplados, cinemática inversa, look-ahead
  entre segmentos) que no aplica a 1 eje lineal.
- `src/axis_controller.h` / `.cpp` — equivalente de `RobotJoint`, pero para 1 eje:
  agrega la conversión mm↔rad según el paso del husillo, que en el original no
  existe (ahí `motor_pos` es directamente la posición angular deseada).
- `src/command_processor.h` / `.cpp` — set de comandos G/M-code reducido a lo que
  necesita 1 eje (`G0/G1 X`, `G28`, `G90/G91`, `M17/M18`, `M55`, `M56`, `M114`,
  `M119`). Reemplaza el switch de `Robot::process_command()` del original.
- `src/shared_data.h` — versión de 1 sola posición del `SharedData` original (que
  tenía arrays de 3 elementos).
- `src/main.cpp` — arranca los dos núcleos y conecta todo. Misma idea arquitectónica
  que el original (Core 0 = comandos + perfil de movimiento a 2kHz, Core 1 = lazo
  servo), pero sin las partes de LED NeoPixel, herramientas PWM (spindle/láser) ni
  demo G-code generator del original, que no hacen falta acá.

## ⚠️ Importante: esto no fue compilado ni probado en hardware

Escribí este código revisando cuidadosamente las firmas y dependencias reales de
cada módulo reusado (leyendo los `.h`/`.cpp` completos del repo original), pero
**no tengo forma de compilarlo en este entorno**: no tengo acceso de red al
toolchain de PlatformIO/arm-none-eabi-gcc para RP2040 desde acá. Tratalo como un
punto de partida sólido, no como código listo para flashear sin revisión. Antes
de subirlo a tu placa:

1. Abrilo con PlatformIO (VSCode o `pio run` por CLI) y **compilalo primero**
   (`pio run -e pico`) para que el compilador marque cualquier error de tipos,
   nombres o includes que se me haya escapado.
2. Revisá `src/hw_config.h` y ajustá con tus valores reales:
   - `LEAD_SCREW_PITCH_MM` (paso de tu husillo)
   - `AXIS_MIN_POSITION_MM` / `AXIS_MAX_POSITION_MM` (recorrido real del carro)
   - `DEFAULT_MAX_VELOCITY_MM_S` / `DEFAULT_MAX_ACCEL_MM_S2` (empezar bajo)
   - `ENCODER_MAGNET_PITCH` / `ENCODER_MAGNET_RADIUS` (tu anillo de imanes)
   - pines, si tu placa no usa exactamente el pinout de `SINGLE_AXIS_BOARD`
3. `POS_KP/KI` y `VEL_KP/KI` son los del proyecto original (tuneados para brazos
   delta livianos) — van a necesitar retuneo con `M55` una vez que el eje mueva.

## Cómo usarlo (una vez compilado y flasheado)

Por serial a 921600 baudios:
```
G28              ; homing contra el tope mecánico
M56 S            ; calibrar y guardar la calibración en flash
M17              ; habilitar el motor
G1 X10 F2        ; mover a 10mm a 2mm/s
M114             ; reportar posición actual
M119             ; reportar estado completo
```

## Lo que falta (no está en este paquete)

- **El encoder lineal del carro y el lazo en cascada** que discutimos antes no
  están implementados acá — es la parte que no existe en el repo original y que
  depende del chip/protocolo específico que vayas a usar. Cuando definas qué
  encoder lineal vas a comprar, lo armamos como un módulo nuevo que corrija el
  `target_position_mm` antes de que `TrapezoidalProfile`/`AxisController` lo usen.
- Logging de alta tasa para la comparación con el interferómetro (repetibilidad,
  backlash) — hoy solo hay `M114`/`M119` como snapshots puntuales.
