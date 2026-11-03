// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "servo_controller.h"
#include "utilities/lookup_table.h"

//*** FUNCTIONS *************************************************************************

bool measure_calibration_data(
  LookupTable& encoder_raw_to_motor_pos_lut,
  LookupTable& motor_pos_to_field_angle_lut,
  ServoController& servo_controller,
  float field_angle_range,
  float field_velocity,
  size_t size,
  bool print_measurements);

// Enables/disables motor-harmonic filtering for the next calibration (M56 F<0|1>).
void set_calibration_motor_filter(bool enable);
bool get_calibration_motor_filter();

// Measures the commutation offset: rotates the field open loop by 'cycles' electrical cycles
// in direction 'sweep_sign' and back while reading the encoder. For each encoder period shift
// k = -1, 0, +1, offset_rad[k+1] receives the mean offset between the commanded field angle and
// the one given by the LUT (rotor lag cancels by averaging both directions). The correct
// reference yields ~0; a one-period shift yields ~120 deg electrical.
// 'consistency' (0..1) measures how constant the offset was during the sweep; a low value means
// the encoder is not tracking the rotor. Field and rotor end where they started.
// Requires the motor to be energized and the servo loop to be stopped.
void measure_commutation_offsets(
  ServoController& servo_controller,
  float sweep_sign,
  float cycles,
  float field_velocity,
  float offset_rad[3],
  float& rotor_lag_rad,
  float& consistency);

