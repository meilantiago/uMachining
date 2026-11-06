# Single-axis ball-screw build: 10 nm steps with the MicroManipulatorStepper firmware

Hi! We are Francisco Maresca and Tiago Meilan. Over the last few weeks we used the `MotionControllerRP` firmware to drive a **single linear axis** for micromachining instead of the delta. This post covers the setup we used, how we got to **10 nm commanded steps that the loop resolves**, what doesn't work yet, and what we'd like to send back as pull requests.

**In short:** we use the same motor, the same MT6835 and the same 60-magnet "magnetic gearing" wheel as your project. We put them on a 5 mm/rev ball screw and changed several parts of the firmware. With that, 10 nm steps show up cleanly in the encoder position (+10, +11, +9 nm at 10, 30 and 200 mm of travel). At rest the position holds within 0.8–1.4 nm (standard deviation), anywhere on 255 mm of travel.

> **Resolution, not accuracy.** As your README warns, these are not the same thing. Every number here is measured by the motor-side encoder, the same way your 50 nm figure is. We have not yet checked the carriage against an external reference. Screw lead error, backlash and nut compliance are invisible to this sensor. An interferometer measurement is planned.

<!-- TODO: photo of the bench (rail on the granite plate, motor end with the magnet wheel and sensor) -->

---

## 1. The setup

| | Ours | Upstream (delta, HW v4) |
|---|---|---|
| Kinematics | 1 linear axis, carriage on a crossed-roller rail | 3-DOF parallel delta |
| Transmission | **Ball screw, 5 mm/rev** | Direct drive, 15 mm rotor radius |
| Output travel per motor radian | **0.80 mm/rad** | ~15 mm/rad at the arm attachment |
| Motor | 1.8° stepper (50 pole pairs), TB6612 | same |
| Encoder | MT6835 facing 60 alternating magnets (Ø3 × 5 mm, N35) at r = 28.9 mm → 30 periods/rev | same principle (r = 30 mm) |
| Encoder count at the output | **0.08 nm** (5 mm / (30 × 2²¹)) | ~1.5 nm at the arm attachment |
| MCU / loop | RP2040, cascaded loop at ~16 kHz on core 1 | RP2040, 30 kHz |
| Travel | 255 mm, homing against the hard stop | — |

The key difference is the screw. Every angular error source maps to output displacement through the mm-per-radian ratio: encoder noise, PWM quantization of the field angle, the residual of the calibration table, the motor's cogging. Going from ~15 mm/rad to 0.8 mm/rad shrinks all of them **about 19×** at the output.

The cost is speed and travel per revolution. Our motion tests are at 2 mm/s. Above ~4 mm/s the motor runs out of torque margin in closed loop with the 0.6 current factor.

We want to be upfront: **a large part of the gain is mechanical.** But the stock firmware could not use it. Section 2 explains why. It would have been capped at 3–24 nm by float precision, and at tens of µm by a limit cycle at rest.

## 2. How we got there

In the order we hit the problems. Each step was measured with 1 kHz telemetry over the full travel.

### 2.1 Multi-turn calibration: the "seam" offset

On a delta the joints turn less than a revolution (`CALIBRATION_RANGE = 83°`). A screw turns ~50 revolutions, so the encoder → motor table has to be periodic over one revolution.

To make it periodic, we measured the offset at the table's edges and spread it over the revolution. That created a ramp in commutation of 0 → 35° electrical within each revolution. The cause: at the edges of the swept range, samples exist in only one direction, so the forward/backward averaging does not cancel the rotor lag there. The carriage stalled in one direction only, always at the same phase of the revolution.

**Fix:** sweep 400°, drop 15° at each end (where the lag hasn't settled), keep only the range covered by both passes, and fold all samples onto one revolution before the least-squares fit. No offset is distributed anymore. The commutation error dropped to ±4° electrical everywhere.

*Relevance upstream: only for multi-turn uses (screws, belts, rotary stages).*

### 2.2 Velocity feed-forward and velocity from the calibrated table

This was the largest single improvement in motion. Following error at cruise, 2 mm/s:

| | Constant lag | Per-magnet ripple | Max \|error\| |
|---|---|---|---|
| Before (velocity from raw counts, no feed-forward) | 13.7 µm | 5.2 µm | 25.3 µm |
| Velocity from the calibrated table only | 13.3 µm | 0.2 µm | 16.7 µm |
| Feed-forward only | 0.7 µm | 5.6 µm | 12.0 µm |
| **Both** | **0.7 µm** | **0.2 µm** | **4.0 µm** |

- **Feed-forward:** the motion profile's velocity is added to the position loop's output. The position PI then only corrects the residual, so the constant lag disappears.
- **Velocity from the table:** differentiating raw encoder counts carries the encoder's nonlinearity into the velocity loop as a ripple that repeats every two magnets. Scaling the count rate by the local slope of the calibration table removes it.

<!-- attach: fig6 (before/after, 4 configurations), relabeled in English -->

### 2.3 `POS_KI`: the limit cycle at rest

With the default `POS_KI = 30000`, the position integral fought static friction. The result was a 40–50 Hz limit cycle at rest, **32 µm peak-to-peak**. That alone hides any nm-scale step.

The velocity loop's integral already drives the steady-state error to zero against a constant load. The position integral only adds correction of slow, velocity-proportional errors while moving (for example, a scale mismatch between the profile and the measured velocity). We swept it:

| `POS_KI` | Error at rest (std) | Following error at 2 mm/s |
|---|---|---|
| 30000 | ~32 µm p-p, 40–50 Hz | — |
| 3000 | ~1 µm p-p | — |
| 1500 | 0.13–0.17 µm, hunting at 2.7 Hz | 1.76 µm RMS |
| **100** | **0.002 µm, no hunting** | 1.79 µm RMS (same) |

Above 200–500 (depending on the day and position) it starts hunting. Below that threshold, motion doesn't change. With 100 the integral time constant is ~0.6 s: it still corrects drift along the rail and leaves the rest state quiet.

*Your mechanics have different friction, so the threshold will differ. The method (sweep with rest periods and look for the hunting onset) should carry over.*

### 2.4 Removing the motor's fingerprint from the calibration, plus learned anti-cogging

The table is built open loop, assuming the rotor follows the commanded field smoothly. It doesn't: within each electrical cycle the rotor leads and lags (detent torque, current shape), and that pattern gets baked into the table. We separate it by frequency:

- The wheel repeats 30·m times per revolution.
- The motor repeats 50·m times per revolution.

So after `M56` we subtract the 50·m harmonics, phased on motor position, and leave the shared ones (150, 300…). On top of that, a small anti-cogging table of field lead per electrical cycle is learned in closed loop at the end of each calibration.

Full travel at 2 mm/s: RMS **1.8 → 1.0 µm**. In a 0–50 mm test, the anti-cogging table took the full-step component (25 µm period) from 1.46 to 0.06 µm.

### 2.5 The float floor

With the loop quiet, the next limit was numeric. Position travelled through the firmware as 32-bit `float`, which has a 24-bit mantissa, so its step grows with the value:

| Where | Float step at the output |
|---|---|
| `float(raw)` into the lookup table (~63 M counts per revolution) | 2.5 nm at 30 mm, 20 nm above ~170 mm |
| `motor_pos` in rad | 3 nm at 30 mm, 24 nm at 204–255 mm |
| Target (`strtof`, profile, telemetry) | 15 nm at 128–255 mm |

At rest we saw the position jumping between levels about 3 nm apart at 30 mm. Near the end of travel the loop effectively had a 24 nm sensor.

**Fix:**
- Whole revolutions are removed in 64-bit integer arithmetic before the table lookup, so the table only sees < 2²⁶ counts.
- `motor_pos`, the target and the position error are `double`. The error itself is small, so it goes back to `float` without loss.
- Position crosses cores as an `int32` in 0.25 nm units, which is a single atomic write.
- Commutation stays in `float`.

Doubles are software-emulated on the M0+. Caching the revolution offset and precomputing the table constants kept the loop at 15.9 iterations/ms, the same as before. `evaluate_periodic_split` takes 6.4 µs, against 7.7 µs for the original `evaluate()`.

| | Before | After |
|---|---|---|
| At rest, 30 mm | ~3 nm quantization steps | 1.4 nm std, 0.25 nm levels |
| At rest, 200 mm | 12–24 nm steps (expected) | 0.8 nm std |
| 10 nm steps at 10, 30, 200 mm | not resolvable near the end | **+10, +11, +9 nm** |

*Relevance upstream: on the delta, joint angles stay within ~±2 rad and counts within ~15 M, so float still resolves a few nm. It is not your bottleneck. It matters for anyone using the firmware on a multi-turn axis.*

## 3. What doesn't work yet

- **Stick-slip after small steps.** At this resolution a friction limit cycle shows up. After a small step, the loop output builds up slowly for ~2 s while the carriage barely moves. Then static friction breaks and the carriage jumps 40–80 nm. It damps out in ~8 s, down to ~1 nm. So a 10 nm step is resolved in steady state, but the transient overshoots several times its size. We are looking at the position integrator's interaction with static friction.
- **Torque margin.** Loop-output peaks touch the ±81° limit a few times per revolution, from mechanical friction that varies along the revolution. The maximum error (~6 µm) comes from there.
- **No external reference yet.** See the note at the top.
- **The encoder mount.** In our current build the sensor sits on a 56 mm cantilever with ~0.3 mm radial play, and the field at the chip is right at the MT6835's 30 mT minimum. We simulated the magnet ring with magpylib and redesigned the wheel (one pocket per magnet, D-bore) and the sensor holder (axial gauge, 0.18 mm shim). Simulated error before the table drops from 15.5 to 3.8 µm p-p, and the minimum field rises from 30 to 38 mT. Not built yet.

## 4. What we'd like to contribute

We'd rather ask than drop a big PR. These are independent pieces, ordered by how useful we think they are to the delta:

1. **Velocity feed-forward from the planner into `ServoController`** (2.2). The planner already knows the joint velocities. On our axis this cut the constant lag from 13.7 to 0.7 µm.
2. **Velocity from the calibration table's slope** (2.2). Small and self-contained.
3. **Tuning notes for `POS_KI`, with our sweep data** (2.3). Possibly a lower default, or just documentation.
4. **Motor-harmonic filtering in `actuator_calibration`** (2.4), optional and off by default. The anti-cogging learner could follow if there's interest.
5. **Multi-turn support**: the calibration seam fix (2.1) and the 64-bit/double position path (2.5), behind a config flag, for people building linear axes.
6. **Telemetry**: a 28-byte binary packet at 1 kHz (`M120`) and the Python tools that consume it.

We have not tested any of this on a delta. Each PR would be ported against current `master` and kept as small as possible.

Our firmware, data and (Spanish) documentation: <https://github.com/meilantiago/uMachining>

Thanks for publishing the project. The calibration approach and the magnetic gearing idea made all of this possible.
