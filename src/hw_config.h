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

// ADJUST: soft travel limits; G0/G1 targets outside this range are rejected.
constexpr float AXIS_MIN_POSITION_MM = 0.0f;
constexpr float AXIS_MAX_POSITION_MM = 255.0f;

//--- MOTION PROFILE (trapezoidal) ----------------------------------------------
constexpr float DEFAULT_MAX_VELOCITY_MM_S = 2.0f;   // [mm/s]
constexpr float DEFAULT_MAX_ACCEL_MM_S2   = 5.0f;   // [mm/s^2]
constexpr float MAX_VELOCITY_LIMIT_MM_S = 5.0f;     // upper bound accepted by M203 X<mm/s>

//--- MOTOR ---------------------------------------------------------------------
// Stepper pole pairs: 50 for 1.8°/step motors, 100 for 0.9°/step.
constexpr float MOTOR_POLE_PAIRS = 50;

// Maximum current factor [0..1]. Below 0.4 the PWM resolution drops significantly.
constexpr float MOTOR_MAX_CURRENT_FACTOR = 0.6f;

//--- ROTARY ENCODER (MT6835 facing a magnet wheel on the motor shaft) ----------
// ADJUST: one encoder period every two magnets (60 magnets = 30 periods per revolution).
constexpr int ENCODER_MAGNET_COUNT = 60;
constexpr int ENCODER_PERIODS_PER_REV = ENCODER_MAGNET_COUNT / 2;
// Encoder -> rotor angle used before a calibration (M56) exists.
constexpr float ENCODER_ANGLE_TO_ROTOR_ANGLE = 1.0f / ENCODER_PERIODS_PER_REV;

// Encoder CRC check. Some chips always return CRC = 0: keep it off unless debugging.
constexpr bool ENABLE_ENCODER_CRC = false;

//--- HOMING (open loop, against the mechanical hard stop) ----------------------
constexpr float HOMING_VELOCITY = 2.0f;             // [motor rad/s], 1.6 mm/s with a 5 mm screw
constexpr float HOMING_SEARCH_RANGE_MM = AXIS_MAX_POSITION_MM + 15.0f;
constexpr float HOMING_CURRENT  = MOTOR_MAX_CURRENT_FACTOR;

//--- CALIBRATION (encoder -> motor LUT, motor -> field LUT) --------------------
// Sweep range from home [deg]; RANGE - 2*SKIP must exceed one revolution.
constexpr float CALIBRATION_RANGE = 400.0f;
// Samples discarded at each end of the sweep, while the rotor lag settles [motor deg].
constexpr float CALIBRATION_TRANSIENT_SKIP = 15.0f;
constexpr float CALIBRATION_FIELD_VELOCITY = 20.0f;
constexpr int   ENCODER_LUT_SIZE = 1024;            // ~0.36° per cell
// Remove the motor's 50*m harmonics per revolution from the table (M56 F0 disables it).
constexpr bool CALIBRATION_FILTER_MOTOR_HARMONICS = true;

//--- COMMUTATION CHECK (after G28 and M56) ------------------------------------
// Homing fixes the encoder zero modulo one encoder period; an open-loop sweep corrects a
// one-period error, which would otherwise shift commutation by 120° or 240°.
constexpr float COMMUTATION_CHECK_CYCLES = 2.0f;    // electrical cycles per pass
constexpr float COMMUTATION_MAX_OFFSET_DEG = 45.0f; // maximum residual offset [electrical deg]

//--- CONTROL LOOP (cascaded PI: position -> velocity) --------------------------
// ADJUST: tune live with M55 before changing these defaults.
constexpr float POS_KP = 60.0f;
// Higher values make the integral hunt against static friction at rest (from ~250 here).
constexpr float POS_KI = 100.0f;

constexpr float VEL_LOWPASS_TC = 0.004f;            // [s]
// With the motor coupled to the carriage, VEL_KP >= 0.16 oscillates at ~200 Hz at rest.
constexpr float VEL_KP = 0.12f;
constexpr float VEL_KI = 54.0f;

// Velocity from the calibration table slope, and profile velocity feed-forward (M58 V F).
constexpr bool VELOCITY_FROM_LUT = true;
constexpr bool VELOCITY_FEEDFORWARD = true;

//--- ANTI-COGGING (M59) --------------------------------------------------------
constexpr bool  COGGING_ENABLED_AT_BOOT = true;     // if a table is stored in flash
constexpr float COGGING_DIR_REF_MM_S = 2.0f;        // learning velocity [mm/s]
constexpr float COGGING_FADE_END = 1.5f;            // fades to 0 at this multiple of the reference
// Relearn at the end of every M56 (M56 A0 skips it): passes 0 -> END_MM -> 0, best one kept.
constexpr bool  COGGING_RELEARN_ON_CALIBRATION = true;
constexpr int   COGGING_RELEARN_PASSES = 2;
constexpr float COGGING_RELEARN_END_MM = 50.0f;
constexpr float COGGING_RELEARN_GAIN = 0.8f;
constexpr float COGGING_RELEARN_ABORT_UM = 150.0f;

//--- PINS (single-axis board) --------------------------------------------------
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
