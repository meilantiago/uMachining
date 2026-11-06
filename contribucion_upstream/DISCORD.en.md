<!-- Mensaje 1 de 2 -->

**Full magnet ring + an encoder placement rule → 10 nm steps (MotionControllerRP on a linear axis)**

Hi all! We're Francisco Maresca and Tiago Meilan. We adapted MotionControllerRP to a single linear axis for micromachining. It now resolves **10 nm steps** (we measured +10, +11 and +9 nm) and holds ~1 nm std at rest.

**The arrangement**
Instead of an arc we use a **full 360° ring**: 60 alternating magnets (Ø3×5 mm N35, r = 28.9 mm), giving 30 field periods per turn, so the motor can turn without limit. The MT6835 board is tangent to the ring, and the die sits past the magnets' end face.

**Where to put the sensor: a rule from the field's derivative**
The chip reads θ = atan2(B_z, B_t). What matters for linearity is its derivative along the travel, which should be constant:
dθ/dx = (B_t·∂B_z/∂x − B_z·∂B_t/∂x) / (B_t² + B_z²)
We simulated the magnets in magpylib, computed dθ/dx over one period, and scored each die position by its coefficient of variation (CV = std/mean).

The CV map over the cross-section around a magnet shows the field's topology:
- The plane of the magnet faces is a band of **field nulls**. B reverses between neighbouring magnets, |B| → 0, dθ/dx spikes and the chip sees 180° jumps.
- Thin **valleys of near-constant dθ/dx** run diagonally out from the magnets' edges. Along them B_t and B_z have equal amplitude, the Lissajous is a circle, and the field vector rotates at a steady rate. In our ring model the valley is a straight line: ~0.36 mm further out for each mm past the face.
- |B| decays fast as you move out along the valley.

The rule: **stay in the valley and get as close to the magnets as the package allows, keeping |B| above the chip's 30 mT minimum with margin.** Ours: ~1.6 mm radial and 1.5 mm past the face, giving CV 1.7·10⁻³, 0.1° error and 57 mT.

<!-- Mensaje 2 de 2 -->

The same model chose the magnets (Ø3×5 vs. Ø5×3, N35 vs. N52). A Monte Carlo over mounting tolerances showed that radial play dominates, which shaped our next wheel.

**Firmware: what gave the resolution**
- POS_KI 30000 → 100: the position integral was fighting static friction, a ~32 µm p-p limit cycle at rest.
- No float in the position path: whole turns in int64 before the LUT, position, target and error in double. Float capped us at 3–24 nm.
- Calibration folded over one full turn (needed for the ring), plus a motor-harmonic filter.

⚠️ This is resolution, not accuracy: everything is measured by the encoder itself. An interferometer check is coming.

Happy to send PRs. Which would help most? https://github.com/meilantiago/uMachining
