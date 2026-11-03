# μMachining

**English** · [Español](README.es.md)

Closed-loop firmware for a single linear micromachining axis, running on an RP2040.

A stepper motor drives a carriage through a ball screw. A magnetic encoder on the motor shaft closes the loop, and the motor is driven with field-oriented control (FOC) instead of steps. The firmware is adapted from the `MotionControllerRP` firmware of [MicroManipulatorStepper](https://github.com/0x23/MicroManipulatorStepper) by 0x23, which drives a 3-DOF delta. Here it drives one multi-turn linear axis.

**Authors:** Francisco Maresca · Tiago Meilan

## Features

- Cascaded position → velocity → field-angle loop at ~16 kHz on a dedicated core
- Automatic encoder and commutation calibration over a full motor revolution, saved to flash
- Removal of the motor's own harmonics from the calibration table
- Anti-cogging table, relearned automatically after each calibration
- Velocity feed-forward from the motion profile, and velocity from the slope of the calibration table
- Position path without float rounding: whole turns in 64-bit integers, position and error in `double`
- Homing against the mechanical hard stop, with an automatic commutation check
- Safety cut-off on following error, with a trace of the last fault
- Binary telemetry at up to 2 kHz, and a PC interface with live plots

## Performance

Measured on our bench at 2 mm/s over the full 255 mm travel:

| | |
|---|---|
| Following error while moving | ~1.0 µm RMS |
| Position noise at rest | ~1 nm (standard deviation) |
| Smallest step resolved | 10 nm |
| Encoder resolution at the carriage | 0.08 nm per count |

> **Resolution, not accuracy.** All of these figures come from the motor-side encoder itself: they compare the measured position with the commanded one. Screw lead error, backlash and nut compliance are invisible to this sensor. The carriage has not yet been measured against an external reference.

## Hardware

| | |
|---|---|
| Board | Raspberry Pi Pico (RP2040) |
| Motor | 1.8° stepper (50 pole pairs) |
| Driver | TB6612 |
| Transmission | Ball screw, 5 mm per revolution |
| Encoder | MT6835 (21-bit) facing a wheel of 60 alternating magnets (30 field periods per revolution) |
| Pinout | Same as the `SINGLE_AXIS_BOARD` option of the upstream firmware (see [`src/hw_config.h`](src/hw_config.h)) |

## Getting started

### 1. Configure

Every parameter that depends on the mechanics is in [`src/hw_config.h`](src/hw_config.h). Check at least the values marked `ADJUST`:

- `LEAD_SCREW_PITCH_MM`: travel per motor revolution
- `AXIS_MIN_POSITION_MM` / `AXIS_MAX_POSITION_MM`: soft travel limits
- `MOTOR_POLE_PAIRS`: 50 for 1.8° motors, 100 for 0.9° motors
- `ENCODER_MAGNET_COUNT`: number of magnets on the wheel
- The pin definitions, if your board differs

### 2. Build and upload

Requires [PlatformIO](https://platformio.org/). Close any serial monitor before uploading.

```bash
pio run -t upload
```

### 3. First run

Connect over USB serial at 921 600 baud and send:

```
M56 S      home, calibrate, relearn anti-cogging and save to flash
M17        enable the closed loop
G1 X10     move to X = 10 mm
M114       report the position
```

`M56` takes a few minutes: it searches for the hard stop, sweeps the field over more than one revolution in each direction, checks the commutation and runs two short back-and-forth passes to learn the anti-cogging table. The carriage needs free travel from 0 to 50 mm.

**Run `M56` on every power-up.** A calibration restored from flash can leave the commutation offset by one encoder period; `M56` measures it again.

### PC tools

```bash
pip install numpy pyserial PyQt5 pyqtgraph

# graphical interface: control and live telemetry
python3 tools/interfaz.py

# send commands in sequence and print the replies
python3 tools/send_gcode.py "M56" "M17" "G1 X10" "M114" --timeout 600
```

`send_gcode.py` defaults to `/dev/ttyACM0` and the interface detects the board automatically; both accept `--port`. The interface can record telemetry and commands to CSV under `mediciones/YYYY-MM-DD/`.

> The interface and the firmware's serial messages are in Spanish. Commands and replies (`ok` / `error: ...`) follow the usual G-code conventions.

## Commands

| Command | Description |
|---|---|
| `G0` / `G1 X<mm> [F<mm/s>]` | Move. Targets outside the soft limits are rejected |
| `G90` / `G91` | Absolute / relative mode |
| `G28 [H]` | Home against the hard stop and check the commutation. `H`: take the current position as home (motor uncoupled only) |
| `M17` / `M18` | Enable the closed loop at the current position / disable the motor. `M17` requires a calibrated and homed axis |
| `M56 [S] [P] [H] [F<0\|1>] [A<0\|1>]` | Home and calibrate. `S`: save to flash · `P`: print the measurements · `H`: calibrate in place · `F`: motor-harmonic filter · `A`: anti-cogging relearn |
| `M114` | Position |
| `M119` | Full status, including the last fault cut-off |
| `M203 [X<mm/s>]` | Maximum velocity for `G0`/`G1` (default 2, up to 5) |
| `M55 A B C D F` | Set the loop gains live: `POS_KP`, `POS_KI`, `VEL_KP`, `VEL_KI`, velocity low-pass time constant |
| `M58 V<0\|1> F<0\|1>` | Velocity from the calibration table / velocity feed-forward |
| `M59 [S<0\|1>] [G<gain>] [C] [P] [W] [L]` | Anti-cogging: enable, gain, clear, print, save to / load from flash |
| `M120 S<0\|1> [P<n>]` | Binary telemetry on/off, one point every `n` ticks of the 2 kHz profile timer |
| `M121` | Dump the trace recorded before the last fault cut-off |
| `M57 X<mm> [F<mm/s>] [P<n>]` | Commutation diagnostic: open-loop sweep printing the commutation offset |

### Telemetry format

Each point is `0xAA 0x56` followed by 28 bytes, little endian (`TelemetryPoint` in [`src/shared_data.h`](src/shared_data.h)):

| Field | Type | Unit |
|---|---|---|
| `timestamp_us` | `uint32` | µs |
| `target_q` | `int32` | 0.25 nm |
| `motor_pos_q` | `int32` | 0.25 nm |
| `vel` | `float` | motor rad/s |
| `out` | `float` | electrical rad (velocity-loop output) |
| `chip_raw` | `int32` | raw MT6835 angle (0 … 2²¹ − 1) |
| `status` | `uint8` | encoder status bits |
| `flags` | `uint8` | bit 0: loop active · bit 1: fault cut-off |
| `seq` | `uint16` | point counter (gaps = dropped points) |

## Architecture

```
Core 0                                  Core 1
──────                                  ──────
serial G-code → CommandProcessor        ServoController (free-running, ~16 kHz)
2 kHz timer → TrapezoidalProfile   ──►    encoder → LUT → position
              target position (int32)     position PI → velocity PI → field angle
telemetry ring buffer → serial     ◄──    TB6612 FOC commutation
```

```
src/
  main.cpp                  boots both cores, serial loop and telemetry
  hw_config.h               mechanics, pins and loop parameters
  axis_controller.*         homing, calibration, commutation check, mm ↔ rad
  command_processor.*       G/M-code commands
  shared_data.h             data shared between the cores, telemetry buffer
  motion_profile/           trapezoidal point-to-point profile
  servo_control/            servo loop, PID, calibration, homing, anti-cogging
  hardware/                 MT6835 encoder and TB6612 driver
  command_parser/           G-code parser
  utilities/                lookup table, fixed-point math, logging
tools/
  interfaz.py               PyQt5 interface with live telemetry
  send_gcode.py             command-line sender
```

### Relation to the upstream firmware

- **Reused or lightly adapted:** the MT6835 and TB6612 drivers, the PID controller, the homing controller, the G-code parser and the logging utilities.
- **Extended:** the servo controller (velocity feed-forward, velocity from the table, anti-cogging, `double` position), the actuator calibration (folding over a full revolution, transient skip at the sweep ends, motor-harmonic filter) and the lookup table (periodic over one revolution, whole turns removed in 64-bit integers).
- **New for the single axis:** the trapezoidal profile (replaces the delta's path planner and kinematics), the axis controller, the command processor, the shared data, the anti-cogging learner and `main.cpp`.

## Known limitations

- **Stick-slip after small steps.** After a step of a few tens of nm the carriage can stay still for ~2 s, then jump 40–80 nm when static friction breaks, and settle to ~1 nm within ~8 s.
- **Speed.** Tested at 2 mm/s. Around 4 mm/s the motor runs out of torque margin with `MOTOR_MAX_CURRENT_FACTOR = 0.6`.
- **Homing direction.** Only homing against the negative hard stop is implemented.
- **Board.** Tested on the RP2040 only.

## License

MIT, following the upstream project. See [LICENSE](LICENSE).

Thanks to 0x23 for publishing MicroManipulatorStepper: its calibration approach and magnetic-gearing encoder made this project possible.
