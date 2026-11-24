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

// Mide el desfase de conmutacion: gira el campo en lazo abierto 'cycles' ciclos electricos
// en el sentido 'sweep_sign' y vuelve, leyendo el encoder. Para cada corrimiento del
// contador del encoder de k = -1, 0, +1 periodos devuelve en offset_rad[k+1] el desfase
// medio entre el campo comandado y el que da la tabla (el atraso del rotor se cancela
// promediando ida y vuelta). Con la referencia correcta da ~0; un periodo corrido da ~120°.
// 'consistency' (0..1) mide cuan constante fue el desfase durante el barrido: bajo indica
// que el encoder no sigue al rotor. El campo y el rotor terminan donde empezaron.
// Requiere el motor energizado y el lazo servo detenido.
// Activa/desactiva el filtrado de los armonicos del motor en la proxima calibracion (M56 F<0|1>).
void set_calibration_motor_filter(bool enable);
bool get_calibration_motor_filter();

void measure_commutation_offsets(
  ServoController& servo_controller,
  float sweep_sign,
  float cycles,
  float field_velocity,
  float offset_rad[3],
  float& rotor_lag_rad,
  float& consistency);

