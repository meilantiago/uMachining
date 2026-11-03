// --------------------------------------------------------------------------------------
// Trapezoidal velocity profile (triangular when the move is too short) for a single
// linear axis. Replaces the multi-axis PathPlanner/MotionController of the original
// MicroManipulatorStepper project by 0x23: here every move is point-to-point, starting
// and ending at zero velocity.
// Positions are double (a float at 255 mm only resolves 15 nm); durations, velocities
// and accelerations are float.
// --------------------------------------------------------------------------------------

#pragma once

//*** CLASS *****************************************************************************

class TrapezoidalProfile {
  public:
    // Start a move from 'start_pos' to 'target_pos' (same units as max_vel/max_accel,
    // typically mm, mm/s, mm/s^2). Replaces any move in progress, starting again from
    // zero velocity (no blending).
    void start(double start_pos, double target_pos, float max_vel, float max_accel);

    // Advance the profile by 'dt' seconds and write the current target position and
    // velocity. Returns true while moving, false once arrived (out_pos = target_pos).
    bool update(float dt, double& out_pos, float& out_vel);

    // true if no move is in progress
    bool is_idle() const;

    // final target of the current (or last) move
    double get_target_position() const;

    // current target (last update() output, or start_pos before the first update())
    double get_current_position() const;

  private:
    double current_pos = 0.0;
    double start_pos = 0.0;
    double target_pos = 0.0;
    float direction = 1.0f;     // +1 or -1
    double distance = 0.0;      // |target_pos - start_pos|

    float max_vel = 0.0f;       // effective cruise velocity (below the requested one for
                                 // short, triangular moves)
    float max_accel = 0.0f;

    float t_accel = 0.0f;       // acceleration ramp duration [s]
    float t_cruise = 0.0f;      // constant-velocity duration [s]
    float t_total = 0.0f;       // total move duration [s]

    double elapsed = 0.0;       // time since start() [s]; double because a float at 100 s
                                // only resolves 8 us (15 nm at 2 mm/s)
    bool active = false;
};
