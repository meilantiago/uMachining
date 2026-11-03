// --------------------------------------------------------------------------------------
// μMachining - hardware and control configuration for a single linear axis.
// Adapted from MicroManipulatorStepper (0x23), firmware/MotionControllerRP
// (SINGLE_AXIS_BOARD variant).
//
// Values marked ADJUST depend on your mechanics and must be checked before first use.
// --------------------------------------------------------------------------------------

#pragma once
#include "utilities/math_constants.h"

//--- LEAD SCREW / MECHANICS ----------------------------------------------------
// ADJUST: linear travel per motor revolution.
constexpr float LEAD_SCREW_PITCH_MM = 5.0f; // [mm/rev]

// Soft travel limits; G0/G1 targets outside this range are rejected.
// ADJUST: keep a margin against the opposite hard stop.
constexpr float AXIS_MIN_POSITION_MM = 0.0f;
constexpr float AXIS_MAX_POSITION_MM = 255.0f;

//--- MOTION PROFILE (trapezoidal) ----------------------------------------------
// Default carriage velocity [mm/s] and acceleration [mm/s^2]. Start low and raise them
// once the closed loop is stable.
constexpr float DEFAULT_MAX_VELOCITY_MM_S = 2.0f;
constexpr float DEFAULT_MAX_ACCEL_MM_S2   = 5.0f;
// Upper bound accepted by M203 X<mm/s>. With velocity feed-forward the limit is the
// motor's torque margin, not the position loop.
constexpr float MAX_VELOCITY_LIMIT_MM_S = 5.0f;

//--- MOTOR ---------------------------------------------------------------------
// Pole pairs of the stepper motor (unrelated to the encoder magnet wheel):
//  * 100 for 0.9°/step motors
//  * 50  for 1.8°/step motors (200 steps/rev)
constexpr float MOTOR_POLE_PAIRS = 50;

// Maximum current factor [0..1]. Below 0.4 the PWM resolution drops significantly.
constexpr float MOTOR_MAX_CURRENT_FACTOR = 0.6f;

//--- ROTARY ENCODER (MT6835 facing a magnet wheel on the motor shaft) ----------
// With alternating N-S magnets the encoder angle completes one 2*PI period every TWO
// magnets: 60 magnets = 30 periods per motor revolution.
// ADJUST if the wheel changes.
constexpr int ENCODER_MAGNET_COUNT = 60;
constexpr int ENCODER_PERIODS_PER_REV = ENCODER_MAGNET_COUNT / 2;
// Encoder angle -> rotor angle before a calibration exists (homing, default table).
// After M56 the measured table is used instead.
constexpr float ENCODER_ANGLE_TO_ROTOR_ANGLE = 1.0f / ENCODER_PERIODS_PER_REV;

// Encoder CRC check (slightly slower, useful for debugging). Some chips always return
// CRC = 0, which produces false errors: try false first.
constexpr bool ENABLE_ENCODER_CRC = false;

//--- HOMING (open loop, against the mechanical hard stop) ----------------------
// Search velocity [motor rad/s]. A lost step looks like the hard stop to the open-loop
// search, so slower (more torque) is safer. 2 rad/s = 1.6 mm/s with a 5 mm screw.
constexpr float HOMING_VELOCITY = 2.0f;
// The search covers the full travel plus a margin, so homing works from any start position.
constexpr float HOMING_SEARCH_RANGE_MM = AXIS_MAX_POSITION_MM + 15.0f;
// Current factor during homing [0..1].
constexpr float HOMING_CURRENT  = MOTOR_MAX_CURRENT_FACTOR;

//--- CALIBRATION (encoder -> motor LUT, motor -> field LUT) --------------------
// Sweep range from home [deg]. The table is periodic over one motor revolution, so the range
// covered in both directions (RANGE - 2*SKIP) must exceed 360°.
constexpr float CALIBRATION_RANGE = 400.0f;         // [deg]
// Samples closer than this to either end of the sweep are discarded [motor deg]: at the start
// of each pass the rotor lag behind the field has not settled yet, and the forward/backward
// average would not cancel it.
constexpr float CALIBRATION_TRANSIENT_SKIP = 15.0f;
constexpr float CALIBRATION_FIELD_VELOCITY = 20.0f; // field velocity during calibration
constexpr int   ENCODER_LUT_SIZE = 1024;             // lookup table size (~0.36° per cell)
// Removes the motor's fingerprint from the encoder -> motor table. The table is built by
// turning the field open loop, assuming the rotor follows it smoothly; within each electrical
// cycle it actually leads and lags (detent torque, current shape), which shows up as 50*m
// harmonics per revolution. The wheel repeats 30*m times per revolution, so harmonics shared
// with it (150, 300, 450...) are kept. Can be disabled per calibration with M56 F0.
constexpr bool CALIBRATION_FILTER_MOTOR_HARMONICS = true;

//--- COMMUTATION CHECK ---------------------------------------------------------
// Homing fixes the encoder zero modulo ONE encoder period (12° of shaft), but commutation
// needs the position modulo one electrical cycle (7.2° of shaft). A home landing on the wrong
// side of a period edge shifts commutation by 240° electrical and the loop stalls saturated.
// After every G28 (and at the end of M56) the field is swept open loop back and forth and the
// real offset is measured; a one-period error (120° or 240°) is corrected.
constexpr float COMMUTATION_CHECK_CYCLES = 2.0f;     // electrical cycles per pass (2 = 14.4° of shaft)
constexpr float COMMUTATION_MAX_OFFSET_DEG = 45.0f;  // maximum accepted residual offset [electrical deg]

//--- CONTROL LOOP (cascaded PI: position -> velocity) --------------------------
// ADJUST: tune live with M55 before changing these defaults.
constexpr float POS_KP = 60.0f;
// With velocity feed-forward, POS_KI barely affects the error while moving; at rest, values
// that are too high make the integral hunt against static friction (here from ~250 upwards).
// It is kept small, rather than zero, to correct slow velocity mismatches while moving.
constexpr float POS_KI = 100.0f;

// Both can be changed live with M58 V<0|1> F<0|1>.
// VELOCITY_FROM_LUT: velocity from the local slope of the calibration table instead of raw
// encoder counts, so the encoder nonlinearity does not reach the velocity loop.
constexpr bool VELOCITY_FROM_LUT = true;
// VELOCITY_FEEDFORWARD: the profile velocity is added to the velocity setpoint, so the
// position PI only corrects the residual (removes the constant following lag).
constexpr bool VELOCITY_FEEDFORWARD = true;
// Anti-cogging field lead table (M59), saved to flash with M59 W. If a table is stored, it is
// loaded at boot and enabled when this is true.
constexpr bool COGGING_ENABLED_AT_BOOT = true;
// Velocity [mm/s] at which the direction-dependent part of the table is learned (M59 R changes it).
constexpr float COGGING_DIR_REF_MM_S = 2.0f;
// The tables are only valid near the learning velocity. Above the reference velocity the
// compensation fades linearly to 0 at COGGING_FADE_END x reference.
constexpr float COGGING_FADE_END = 1.5f;
// Relearn the tables at the end of every M56 (skipped with M56 A0; with M56 H only with A1, since
// free travel is not guaranteed). Runs COGGING_RELEARN_PASSES back-and-forth passes
// 0 -> COGGING_RELEARN_END_MM -> 0 at COGGING_DIR_REF_MM_S and keeps the table of the pass with
// the lowest error. M56 S also saves it to flash.
constexpr bool  COGGING_RELEARN_ON_CALIBRATION = true;
constexpr int   COGGING_RELEARN_PASSES = 2;
constexpr float COGGING_RELEARN_END_MM = 50.0f;
constexpr float COGGING_RELEARN_GAIN = 0.8f;
constexpr float COGGING_RELEARN_ABORT_UM = 150.0f;   // aborts the pass above this error [um]

constexpr float VEL_LOWPASS_TC = 0.004f;
constexpr float VEL_KP = 0.2f;
constexpr float VEL_KI = 90.0f;

//--- PINS (single-axis board) --------------------------------------------------
// Same pinout as the SINGLE_AXIS_BOARD option of the original hw_config.h.
// ADJUST if your board uses a different mapping.
#define PIN_BUILTIN_LED   25
#define PIN_USER_BUTTON   24

#define PIN_M1_PWM_A_POS  2
#define PIN_M1_PWM_A_NEG  3
#define PIN_M1_PWM_B_POS  5
#define PIN_M1_PWM_B_NEG  6

#define PIN_MOTOR_EN      8
#define PIN_MOTOR_PWM_A   4
#define PIN_MOTOR_PWM_B   7

#define PIN_ENCODER1_CS   17
#define PIN_ENCODER_SCK   18
#define PIN_ENCODER_MISO  20
#define PIN_ENCODER_MOSI  19
