// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "utilities/lookup_table.h"
#include "pid.h"

//*** CLASS *****************************************************************************

class ServoController {
  public:
    // use typedefs instead of virtual functions for speed
    typedef TB6612MotorDriver MOTOR_DRIVER_TYPE;
    typedef MT6835Encoder ENCODER_TYPE;

  public:
    ServoController(MOTOR_DRIVER_TYPE& motor_driver, ENCODER_TYPE& encoder, int32_t motor_pole_pairs);

    // initialize the servo controller hardware
    void init(float max_motor_amplitude);

    // set the encoder raw angle to motor position lookup table
    void set_enc_to_pos_lut(LookupTable& lut);
    // get the encoder raw angle to motor position lookup table
    const LookupTable& get_enc_to_pos_lut() const;

    // set the motor position to field angle lookup table
    void set_pos_to_field_lut(LookupTable& lut);
    // get the motor position to field angle lookup table
    const LookupTable& get_pos_to_field_lut() const;

    // Updates the servo loop. velocity_ff: profile velocity [rad/s] added to the position loop
    // output (velocity feed-forward; only if feedforward_enabled).
    // Positions are double: at 255 mm (320 rad) a float only resolves 3e-5 rad = 24 nm.
    void update(double target_motor_pos,
                float dt,
                float one_over_dt,
                float velocity_ff = 0.0f);

    // M58: velocity estimation method and velocity feed-forward enable (see hw_config.h)
    volatile bool velocity_from_lut = true;
    volatile bool feedforward_enabled = true;

    // Anti-cogging feed-forward (M59). COGGING_TABLE_SIZE points over one electrical cycle, indexed
    // by the absolute electrical field angle (independent of homing and calibration). Added as field
    // lead [electrical rad] to the PI output; 'output' (and telemetry) remains the PI output only.
    static constexpr int COGGING_TABLE_SIZE = 64;
    float cogging_table[COGGING_TABLE_SIZE] = {};       // direction-independent part [electrical rad]
    // Part that changes sign with the direction of travel: the loop responds differently to the same
    // pattern in each direction (forward and backward see it with conjugate phases). Scaled by
    // profile velocity / cogging_dir_ref, clamped to +-1 (0 at standstill, smooth on start/stop).
    float cogging_table_dir[COGGING_TABLE_SIZE] = {};
    volatile float cogging_dir_ref = 1.0f;              // motor velocity [rad/s] at which the table was learned
    volatile bool cogging_enabled = false;
    volatile float cogging_gain = 1.0f;
    float cogging_feedforward(float field_angle, float velocity_ff) const;

    // Checks if the motor is at position (uses values from previous update() call).
    bool at_position(float motor_pos_eps);

    // Reads the current motor position from the encoder.
    double read_position();

    // Returns the motor position.
    double get_position();

    // Returns the current position error from the last servo loop update.
    float get_position_error();

    // Velocity estimate and loop output (field angle offset) from the last update.
    float get_velocity() const { return velocity; }
    float get_output() const { return output; }
    bool is_motor_update_enabled() const { return motor_update_enabled; }

    // Moves to a new motor position using closed loop control (blocking).
    bool move_to(float target_motor_angle, 
                 float at_pos_motor_angle_eps, 
                 float settle_time_ms,
                 float timeout_us);

    // Moves to a new motor position using open loop control (blocking).
    // Motor updates must be disabled if servo loop is running in background.
    void move_to_open_loop(float target_motor_angle, 
                           float angular_velocity);
    
    // Returns the encoder object.
    ENCODER_TYPE& get_encoder();

    // Returns the motor driver object.
    MOTOR_DRIVER_TYPE& get_motor_driver();

    // Returns number of motor pole pairs.
    // Can be used to approximate conversion of motor position to field angle.
    float get_pole_pair_count();

    // enable or disable motor 
    void set_motor_enabled(bool enable, bool synchronize_field_angle);

    // enable or disable motor updates
    void set_motor_update_enabled(bool enable);

    // enable or disable encoder reads
    void set_encoder_update_enabled(bool enable);

    // Re-reads the encoder and resets previous position, velocity filter and PIDs. Call with the
    // servo loop stopped after moving the motor outside update() (homing, calibration), so the
    // first update() does not see a huge jump in counts.
    void resync();

  public:
    double encoder_angle_to_motor_pos(ENCODER_TYPE::AbsRawAngleType encoder_angle_raw);
    float motor_pos_to_field_angle(float motor_pos);
    float motor_pos_to_field_angle_derivative(float motor_pos);

  public:
    LowpassFilter velocity_lowpass;
    PIDController pos_controller;
    PIDController velocity_controller;

  private:
    ENCODER_TYPE& encoder;
    MOTOR_DRIVER_TYPE& motor_driver;
    LookupTable encoder_raw_to_motor_pos_lut;
    LookupTable motor_pos_to_field_angle_lut;

    float motor_pole_pair_count = 0.0f;   // number of motor pole pairs (as float to avoid repeated conversion)
    float motor_current_amplitude = 0.5f; // motor current in range [0..1]
    double motor_pos = 0;                 // current motor position
    double motor_pos_prev = 0;            // previous motor position
    // whole LUT periods of the last reading and their offset [rad]: changes once per revolution,
    // so the loop avoids a double multiplication on every pass
    int32_t lut_periods = INT32_MIN;
    double lut_periods_offset = 0.0;
    ENCODER_TYPE::AbsRawAngleType encoder_angle_raw_prev = 0; // previous encoder reading
    float pos_error = 0;                  // current position error as computed by update() (difference of doubles)
    float velocity = 0;                   // current velocity estimate
    float output = 0.0f;                  // servo loop output (field angle offset)
    volatile bool motor_update_enabled = false; // enables motor field updates (written by Core 0, read by Core 1)
    bool encoder_update_enabled = true;   // enables encoder reads
};