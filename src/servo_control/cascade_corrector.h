// --------------------------------------------------------------------------------------
// LinearCascadeCorrector: slow PI controller for an outer cascade loop.
// Compares the commanded position (trapezoidal profile output) with an external linear
// encoder mounted on the carriage and produces an additive correction that compensates
// backlash, thermal drift, mechanical deflection, etc.
//
// Signal flow:
//   TrapezoidalProfile.update() -> pos_mm (reference)
//   read_linear_encoder()       -> linear_pos_mm (measurement)
//   LinearCascadeCorrector.update(pos_mm, linear_pos_mm, dt) -> corrected_pos_mm
//   SharedData.target_position_mm = corrected_pos_mm
//
// Formula:
//   Error  = target_pos - linear_pos
//   Output = target_pos + Kp * Error + Ki * Integral(Error)
//
// The integrator is clamped (anti-windup). Call reset() when homing or calibrating.
// --------------------------------------------------------------------------------------

#pragma once

class LinearCascadeCorrector {
  public:
    LinearCascadeCorrector() = default;

    // Sets the PI gains and the anti-windup limit.
    //   integral_limit:  symmetric clamp of the integral accumulator [mm*s]
    void set_gains(float kp, float ki, float integral_limit);

    // Returns the corrected position [mm] (target + PI correction).
    //   target_pos:  current target position [mm] (trapezoidal profile output)
    //   linear_pos:  measured linear encoder position [mm]
    //   dt:          time since the last call [s]
    float update(float target_pos, float linear_pos, float dt);

    // Clears the integral accumulator. Call before homing (G28) and calibration (M56).
    void reset();

  private:
    float kp_ = 0.5f;              // conservative default
    float ki_ = 2.0f;
    float integral_limit_ = 1.0f;  // anti-windup clamp

    float integral_ = 0.0f;        // integral of the error [mm*s]
};
