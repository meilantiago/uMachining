# μMachining

[Español](README.md) · **English**

Closed-loop control firmware for a linear micromachining axis, running on an RP2040.

A stepper motor drives a carriage through a ball screw. A magnetic encoder on the motor shaft closes the loop, and the motor is driven with field-oriented control (FOC) rather than by steps. The firmware is adapted from [MicroManipulatorStepper](docs/README_ORIGEN_DEL_CODIGO.md).

**Authors:** Francisco Maresca · Tiago Meilan

## Features

- Cascaded position → velocity → field loop at ~20 kHz on a dedicated core
- Automatic encoder and commutation calibration, saved to flash
- Velocity feed-forward from the motion profile
- Homing against the mechanical end stop, with commutation check
- Safety cut-off on following error
- 1 kHz binary telemetry and Python tools for testing, gain sweeps and error analysis

## Performance

Full travel (0–255 mm) at 2 mm/s:

| | |
|---|---|
| Following error while moving | 1.2–1.9 µm RMS (max. 4–8 µm) |
| Error at rest | 0.002 µm (standard deviation, `POS_KI = 100`) |
| Encoder resolution | 0.08 nm per count |

> These are following errors: the position measured by the encoder versus the commanded one. The carriage's absolute accuracy has not yet been measured against an external reference.

## Hardware

| | |
|---|---|
| Board | Raspberry Pi Pico (RP2040) |
| Motor | 1.8° stepper (50 pole pairs) |
| Driver | TB6612 |
| Transmission | Ball screw, 5 mm per revolution |
| Encoder | 21-bit MT6835 facing a 60-magnet wheel |

## Quick start

```bash
# dependencies for the PC tools
pip install numpy matplotlib pyserial PyQt5 pyqtgraph

# build and upload (close the serial monitor first)
pio run -t upload
```

Over serial at 921,600 baud:

```
M56        homing + calibration
M17        enable the closed loop
G1 X10     go to X = 10 mm
M114       current position
```

Or with the graphical interface ([user guide, in Spanish](docs/GUIA_INTERFAZ.md)):

```bash
python3 tools/interfaz.py
```

Or with the script:

```bash
python3 tools/send_gcode.py "M56" "M17" "G1 X10" "M114" --timeout 600
```

## Commands

| Command | Description |
|---|---|
| `G28` | Homing against the end stop |
| `M56 [S]` | Homing + calibration (`S`: save to flash) |
| `M17` / `M18` | Enable / disable the motor |
| `G1 X<mm> [F<mm/s>]` | Move (0–255 mm, up to 2 mm/s) |
| `G90` / `G91` | Absolute / relative mode |
| `M114` / `M119` | Position / full status |
| `M55 A B C D F` | Live PID gains |
| `M58 V<0\|1> F<0\|1>` | Velocity from the calibration table / feed-forward |
| `M120 S1 P<n>` | Binary telemetry |
| `M57 X<mm>` / `M121` | Commutation diagnostics / trace of the last fault cut-off |

## Tools

| Script | Description |
|---|---|
| `tools/interfaz.py` | Graphical interface: remote control and live telemetry ([guide](docs/GUIA_INTERFAZ.md)) |
| `tools/send_gcode.py` | Send commands in sequence |
| `tools/closed_loop_test.py` | Back-and-forth runs with telemetry and plots over the full travel |
| `tools/pid_sweep.py` | Live gain sweep, with error while moving and at rest |
| `tools/ki_sweep.py` | Sweep of a `hw_config.h` constant, rebuilding for each value |
| `tools/analisis_errores.py` | Breakdown of the error into its components |
| `tools/encoder_sweep.py` | Encoder characterization |

Results are saved under `mediciones/YYYY-MM-DD/`.

## Layout

```
src/            firmware (hw_config.h: mechanical setup and PID parameters)
tools/          Python PC scripts
docs/           documentation, reports and changelog
mediciones/     data and plots for each test, by day
simulaciones/   model of the wheel's magnetic field
```

## Notes

- **Run `M56` on every boot, not `G28`.** Calibration restored from flash can leave the commutation offset.
- **`POS_KI = 100`.** Holds the carriage still at rest (0.002 µm) without affecting motion; above ~200–250 the integral term oscillates against static friction ([rationale](docs/ELECCION_POS_KI.md)).
- **Workflow:** `master` is always a stable, tagged version; each experiment lives on a `prueba/<name>` branch.

## Documentation (in Spanish)

- [Firmware architecture](docs/DOCUMENTACION_ARQUITECTURA.md)
- [Encoder and calibration](docs/DOCUMENTACION_ENCODER_Y_CALIBRACION.md)
- [Closed-loop test report](docs/REPORTE_PRUEBA_LAZO_CERRADO_2026-10-16.md)
- [Error analysis method and characterization plan](docs/METODO_DE_ERRORES_Y_PLAN_DE_CARACTERIZACION.md)
- [Choosing POS_KI](docs/ELECCION_POS_KI.md)
- [Changelog](docs/CAMBIOS_2026-10-16.md) (and the earlier `CAMBIOS_*.md`)

## Credits

Based on the `MotionControllerRP` firmware from MicroManipulatorStepper (MIT license). See [what was reused and what was rewritten](docs/README_ORIGEN_DEL_CODIGO.md).
