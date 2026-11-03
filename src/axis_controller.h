// --------------------------------------------------------------------------------------
// AxisController: simplified counterpart of 'RobotJoint' from MicroManipulatorStepper (0x23),
// for a single linear axis (carriage + ball screw) instead of a delta-arm rotary joint.
// Responsibilities:
//   - initialize the MT6835 encoder and TB6612 driver
//   - convert between carriage position [mm] and motor angle [rad] via LEAD_SCREW_PITCH_MM
//   - homing against the mechanical end stop (HomingController)
//   - calibration of the encoder->motor and motor->field lookup tables (measure_calibration_data())
//   - persisting those LUTs to flash (LittleFS)
// --------------------------------------------------------------------------------------

#pragma once

#include <string>
#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "servo_control/servo_controller.h"

//*** CLASS *****************************************************************************

// Unit of the position shared between cores and sent in telemetry: 0.25 nm (int32, +-536 mm).
// A double read from the other core can tear (half old, half new); an int32 is written atomically.
constexpr double POSITION_LSB_MM = 0.25e-6;

class AxisController {
  public:
    AxisController(MT6835Encoder* encoder, TB6612MotorDriver* motor_driver,
                   float motor_pole_pairs, float lead_screw_pitch_mm);
    ~AxisController();

    // Initialize encoder and driver, and load the stored calibration if present.
    void init();

    // Home against the mechanical end stop; direction_sign (+1/-1) is the search direction.
    // BLOCKING: must not run concurrently with update_servo() (disable the Core 1 update first,
    // see homing_controller.h). search_stop = false homes at the current rotor position without
    // seeking the stop (for testing with the motor decoupled).
    // If calibrated, commutation is verified afterwards and the encoder period corrected
    // (see verify_commutation); homing fails on mismatch.
    bool home(float direction_sign, bool search_stop = true);

    // Full calibration sequence (homing + LUT measurement).
    // store_to_flash: save the LUTs to LittleFS so they survive a reset.
    bool calibrate(float homing_direction_sign, bool store_to_flash, bool print_measurements,
                   bool search_stop = true);

    // One servo loop iteration; call as often as possible (dedicated Core 1 loop).
    // target_position_mm [mm]; target_velocity_mm_s [mm/s] is the profile velocity for feed-forward.
    void update_servo(double target_position_mm, float dt, float one_over_dt, float target_velocity_mm_s = 0.0f);

    void set_motor_enabled(bool enable);

    // Carriage position [mm] from the last encoder reading used by update_servo() (no new read).
    // Safe to call from the other core.
    double get_position_mm() const;
    // Same position in POSITION_LSB_MM units (for telemetry)
    int32_t get_position_q() const { return position_q; }

    // Position error of the last loop update [mm]
    float get_position_error_mm() const;

    bool is_homed() const;
    bool is_calibrated() const;

    // true if the loop tripped on sustained following error (see update_servo)
    bool has_servo_fault() const { return servo_fault; }

    // Loop state at the last fault trip (reported by M119)
    struct FaultInfo {
      uint32_t count = 0;   // trips since boot
      float target_mm = 0.0f, pos_mm = 0.0f, vel = 0.0f, out = 0.0f;
      uint8_t status = 0;   // encoder status bits
    };
    const FaultInfo& get_last_fault() const { return last_fault; }

    // Print the closed-loop trace as CSV over serial: one sample per ms over the last ~2 s
    // with the loop active. The trace freezes when a fault trips.
    void dump_trace();

    // Diagnostic (M57): with the servo loop stopped, rotate the field open-loop over
    // 'distance_mm' [mm] (one way) and print as CSV, every 'decimation' steps, the offset between
    // the commanded field and the table's field for the encoder position.
    // Offset = commutation error +/- rotor lag.
    void commutation_sweep(float distance_mm, float speed_mm_s, int decimation);

    // Anti-cogging compensation table (M59): store/load in flash.
    bool store_cogging();
    bool load_cogging();
    // Field angle [electrical rad] at motor position 0, so the PC can compute the electrical
    // angle of each telemetry sample.
    float field_angle_at_zero();

    // Direct access to the internal ServoController, e.g. for live PID tuning (M55).
    ServoController* get_servo_controller();

    // Encoder access for diagnostics (M119: raw angle, CRC errors, ...)
    MT6835Encoder* get_encoder();

  private:
    // Measure open-loop the offset between field and calibration table (motor energized, servo
    // loop stopped). allow_period_correction: if the offset shows the encoder counter is off by one
    // period (120°/240° electrical), fix it; otherwise accept the counter as is.
    // Returns false if the offset is out of tolerance.
    bool verify_commutation(float away_from_stop_sign, bool allow_period_correction);

    float mm_to_motor_pos_rad(float pos_mm) const;
    float motor_pos_rad_to_mm(float motor_pos_rad) const;

    // Convert the servo position and publish it in position_q
    void publish_position();

    std::string enc_to_pos_lut_filename() const;
    std::string pos_to_field_lut_filename() const;

    bool load_calibration();
    bool store_calibration();

  private:
    MT6835Encoder* encoder;
    TB6612MotorDriver* motor_driver;
    ServoController* servo_controller;

    float lead_screw_pitch_mm;
    double mm_to_rad = 0.0;               // 2*pi / pitch [rad/mm] (double: the loop math is done in double)
    double rad_to_q = 0.0;                // motor rad -> POSITION_LSB_MM units
    volatile int32_t position_q = 0;      // carriage position in POSITION_LSB_MM units

    bool homed = false;
    bool calibrated = false;
    volatile bool servo_fault = false;
    uint64_t fault_since_us = 0;
    FaultInfo last_fault;

    struct TracePoint {
      uint32_t t_us;
      float target_mm, pos_mm, vel, out;
      int32_t chip_raw;
      uint16_t loops;   // loop iterations since the previous sample
      uint8_t status;
    };
    static constexpr int TRACE_SIZE = 2000;
    TracePoint trace[TRACE_SIZE];
    volatile int trace_head = 0;
    volatile int trace_count = 0;
    volatile bool trace_frozen = false;
    uint64_t last_trace_us = 0;
    uint16_t loops_since_trace = 0;
};
