// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include <algorithm>
#include <Arduino.h>

#include "servo_controller.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"

#include "actuator_calibration.h"
#include "hw_config.h"

//*** FUNCTION ***********************************************************************************/

// RMS del error de la LUT descontando el error medio de cada sentido de giro (muestras
// [0, split) = ida, [split, fin) = vuelta). 'half_gap' devuelve la mitad de la
// diferencia entre ambos sentidos, o sea el atraso del rotor respecto del campo.
static float rms_error_without_direction_offset(const LookupTable& lut,
                                                const std::vector<std::pair<float, float>>& pairs,
                                                size_t split, float& half_gap)
{
  double sum[2] = {0.0, 0.0}, sum_sq[2] = {0.0, 0.0};
  size_t n[2] = {0, 0};
  for (size_t i = 0; i < pairs.size(); i++) {
    int g = (i < split) ? 0 : 1;
    double e = lut.evaluate(pairs[i].first) - pairs[i].second;
    sum[g] += e;
    sum_sq[g] += e * e;
    n[g]++;
  }

  double var = 0.0;
  for (int g = 0; g < 2; g++)
    if (n[g] > 0)
      var += sum_sq[g] - sum[g] * sum[g] / n[g];

  half_gap = (n[0] > 0 && n[1] > 0) ? float((sum[0] / n[0] - sum[1] / n[1]) * 0.5) : 0.0f;
  return float(sqrt(var / (n[0] + n[1])));
}

bool measure_calibration_data(
            LookupTable& encoder_raw_to_motor_pos_lut,
            LookupTable& motor_pos_to_field_angle_lut,
            ServoController& servo_controller,
            float calibration_range,
            float field_velocity,
            size_t table_size,
            bool print_measurements)
{
  const float max_rmse_rad = Constants::DEG2RAD * 0.5f;
  const float max_weak_field_fraction = 0.05f; // abort if >5% of samples report a weak magnetic field
  const int sample_decimation = 8; // registrar 1 de cada 8 pasos internos (~4ms entre muestras)

  LOG_INFO("Measuring motor to encoder angle lookup table...");

  // get required values
  std::vector<std::pair<float, float>> encoder_angle_and_motor_pos;

  auto& motor_driver = servo_controller.get_motor_driver();
  float pole_pair_count = servo_controller.get_pole_pair_count();
  float start_field_angle = motor_driver.get_field_angle();
  float total_sweep_angle = calibration_range*pole_pair_count; // barrido total en angulo electrico

  // reservar de una: con una vuelta completa son ~8000 muestras y un realloc a mitad
  // del barrido necesitaria el doble de memoria por un instante
  float sweep_time_s = total_sweep_angle / field_velocity;
  encoder_angle_and_motor_pos.reserve(size_t(2.2f * sweep_time_s / (500e-6f * sample_decimation)));

  auto& encoder = servo_controller.get_encoder();

  encoder.get_crc_error_count(true); // reset crc error count
  bool was_crc_enabled = encoder.is_crc_enabled();
  encoder.set_crc_enabled(true);

  int weak_field_measurements = 0;

  // Un unico movimiento continuo por pasada (ida/vuelta), muestreado sobre la
  // marcha via el callback on_step de rotate_field(), sin frenar entre muestras.
  auto run_measurement = [&](float field_angle_delta) {
    int step_counter = 0;
    auto on_step = [&]() {
      // leer siempre (el desenrollado del encoder necesita lecturas frecuentes),
      // pero registrar solo 1 de cada 'sample_decimation'
      float encoder_angle_raw = encoder.read_abs_angle_raw();
      if ((step_counter++ % sample_decimation) != 0)
        return;

      float field_angle = motor_driver.get_field_angle();
      float motor_pos = (field_angle-start_field_angle)/pole_pair_count;
      // TODO: read motor_pos from precise reference encoder

      if(encoder.get_status() & MT6835_STATUS_WEAKFIELD)
        weak_field_measurements++;

      if(print_measurements) {
        LOG_RAW("%15.10f, %15.10f, %f", motor_pos, field_angle, encoder_angle_raw);
        // Formato Teleplot, para ver en vivo donde dentro del barrido aparece el campo debil.
        Serial.printf(">motor_pos_deg:%.4f\n>encoder_angle_raw:%.1f\n>weak_field:%d\n",
                      motor_pos*Constants::RAD2DEG, encoder_angle_raw,
                      (encoder.get_status() & MT6835_STATUS_WEAKFIELD) ? 1 : 0);
      }

      encoder_angle_and_motor_pos.push_back({encoder_angle_raw, motor_pos});
    };

    motor_driver.rotate_field(field_angle_delta, field_velocity, on_step);
  };

  LOG_DEBUG("Running foreward pass...");
  run_measurement(total_sweep_angle);
  size_t forward_sample_count = encoder_angle_and_motor_pos.size();
  LOG_DEBUG("Running backward pass...");
  run_measurement(-total_sweep_angle);

  // rotate back to start position (sin exagerar la velocidad: un patinazo del rotor
  // podria hacer que el encoder pierda el seguimiento)
  motor_driver.rotate_field(start_field_angle-motor_driver.get_field_angle(),
    3.0f*field_velocity, [&servo_controller]() {
      servo_controller.get_encoder().read_abs_angle_raw();
  });

  encoder.set_crc_enabled(was_crc_enabled);

  int total_measurements = (int)encoder_angle_and_motor_pos.size();

  uint32_t encorder_crc_errors = encoder.get_crc_error_count(false);
  if(encorder_crc_errors > 0) {
    LOG_WARNING("Data error (CRC) in %i of %i measurements", encorder_crc_errors, total_measurements);
    // return false;
  }

  if(weak_field_measurements > total_measurements*max_weak_field_fraction) {
    LOG_ERROR("Magnetic field too weak for %i of %i measurements. Check the encoder-to-magnet air gap/magnet strength.",
               weak_field_measurements, total_measurements);
    return false;
  }

  // build lookup table 'encoder_raw_angle -> motor_pos'
  float rmse = 0.0f;
  bool ok = encoder_raw_to_motor_pos_lut.init_interpolating(encoder_angle_and_motor_pos, table_size, true);
  if(ok == false) {
    LOG_ERROR("Creating lookup table 'encoder_raw_angle -> motor_pos' failed.");
    return false;
  }
  encoder_raw_to_motor_pos_lut.optimize_lut(encoder_angle_and_motor_pos, rmse);

  // motor_pos sale del campo comandado, y el rotor atrasa respecto del campo por la
  // friccion: en la ida el motor_pos queda por delante de la posicion real y en la
  // vuelta por detras. La LUT combinada promedia ambos sentidos (lo correcto), pero su
  // rms_error incluye ese atraso, que no es un error de la tabla. Para juzgar la
  // calidad de la tabla se descuenta el atraso medio de cada sentido.
  float rotor_lag = 0.0f;
  rmse = rms_error_without_direction_offset(encoder_raw_to_motor_pos_lut, encoder_angle_and_motor_pos,
                                            forward_sample_count, rotor_lag);
  LOG_INFO("LUT 'encoder_raw_angle -> motor_pos': rms_error = %f deg, atraso del rotor = %f deg",
           Constants::RAD2DEG*rmse, Constants::RAD2DEG*rotor_lag);
  if(rmse > max_rmse_rad) {
    LOG_ERROR("Fitting error of lookup table 'encoder_raw_angle -> motor_pos' unusually high (rms_error = %f deg). "
                "Calibration aborted.",
                Constants::RAD2DEG*rmse);
    return false;
  }

  // Hacer la tabla periodica para cubrir todo el recorrido del husillo (muchas vueltas)
  // con una calibracion de una sola vuelta. La rueda es un anillo cerrado, asi que una
  // vuelta del motor son exactamente N periodos del encoder, con N entero.
  double s_m = 0.0, s_e = 0.0, s_mm = 0.0, s_me = 0.0;
  for (const auto& p : encoder_angle_and_motor_pos) {
    s_m += p.second; s_e += p.first; s_mm += double(p.second)*p.second; s_me += double(p.second)*p.first;
  }
  double n = double(total_measurements);
  double counts_per_motor_rad = (n*s_me - s_m*s_e) / (n*s_mm - s_m*s_m);
  double periods_per_rev = fabs(counts_per_motor_rad) * Constants::TWO_PI_F / MT6835_CPR;
  long ring_periods = lround(periods_per_rev);
  LOG_INFO("Rueda magnetica: %f periodos de encoder por vuelta de motor", periods_per_rev);
  if (ring_periods < 1 || fabs(periods_per_rev - ring_periods) > 0.2) {
    LOG_ERROR("Periodos por vuelta no enteros: el encoder perdio el seguimiento. Calibration aborted.");
    return false;
  }
  if (ring_periods != ENCODER_PERIODS_PER_REV) {
    LOG_ERROR("Se midieron %li periodos por vuelta y la rueda de %i imanes deberia dar %i. "
              "Revisar ENCODER_MAGNET_COUNT y MOTOR_POLE_PAIRS en hw_config.h. Calibration aborted.",
              ring_periods, ENCODER_MAGNET_COUNT, ENCODER_PERIODS_PER_REV);
    return false;
  }
  float lut_in_min, lut_in_max;
  encoder_raw_to_motor_pos_lut.get_intput_range(lut_in_min, lut_in_max);
  if (lut_in_max - lut_in_min < float(ring_periods) * MT6835_CPR) {
    LOG_ERROR("La calibracion no cubre una vuelta completa (subir CALIBRATION_RANGE). Calibration aborted.");
    return false;
  }
  // Empalmar la tabla consigo misma: una vuelta son exactamente ring_periods periodos,
  // pero la pendiente medida sale un poco distinta (cada pasada arranca desde parado y
  // el atraso del rotor tarda en establecerse). Sin corregir, la posicion salta ~0.6
  // grados en la costura de cada vuelta, y el ruido del encoder alrededor de la costura
  // (el home queda justo ahi) se veia como velocidades enormes. Se reparte el desfase
  // linealmente sobre la vuelta para que LUT(min + periodo) = LUT(min) + 2*pi.
  const float period_in = float(ring_periods) * MT6835_CPR;
  const float period_out = counts_per_motor_rad > 0.0 ? Constants::TWO_PI_F : -Constants::TWO_PI_F;
  float seam_error = encoder_raw_to_motor_pos_lut.evaluate(lut_in_min + period_in)
                     - encoder_raw_to_motor_pos_lut.evaluate(lut_in_min) - period_out;
  LOG_INFO("Desfase en la costura de la vuelta: %f deg (se corrige)", Constants::RAD2DEG*seam_error);
  int lut_size = int(encoder_raw_to_motor_pos_lut.size());
  for (int i = 0; i < lut_size; i++) {
    float x = lut_in_min + (lut_in_max - lut_in_min) * i / (lut_size - 1);
    encoder_raw_to_motor_pos_lut.set_entry(i, encoder_raw_to_motor_pos_lut.get_entry(i)
                                              - seam_error * (x - lut_in_min) / period_in);
  }
  encoder_raw_to_motor_pos_lut.set_period(period_in, period_out);

  // 'motor_pos -> field_angle' es lineal por construccion (motor_pos se definio como
  // (campo - campo_inicial)/pares_de_polos). Se arma sobre un rango mayor que todo el
  // recorrido (260 mm = 817 rad de motor con el husillo de 2 mm).
  const float field_lut_range = 1000.0f;
  build_linear_lut(motor_pos_to_field_angle_lut, -field_lut_range, field_lut_range,
                   start_field_angle - field_lut_range*pole_pair_count,
                   start_field_angle + field_lut_range*pole_pair_count);

  LOG_INFO("finished");
  return true;
}

static float wrap_angle(float a) {
  return remainderf(a, Constants::TWO_PI_F);  // lleva a [-pi, pi]
}

void measure_commutation_offsets(
            ServoController& servo_controller,
            float sweep_sign,
            float cycles,
            float field_velocity,
            float offset_rad[3],
            float& rotor_lag_rad,
            float& consistency)
{
  auto& motor_driver = servo_controller.get_motor_driver();
  auto& encoder = servo_controller.get_encoder();
  const float sweep = cycles * Constants::TWO_PI_F * (sweep_sign >= 0.0f ? 1.0f : -1.0f);
  // al arrancar cada pasada el rotor queda pegado hasta que el campo le saca ventaja:
  // no se registra el primer medio ciclo
  const float settle = Constants::PI_F;

  // suma de vectores unitarios del desfase, por pasada (ida/vuelta) y por corrimiento k
  double c[2][3] = {}, s[2][3] = {};
  int n[2] = {0, 0};

  encoder.read_abs_angle_raw();
  for (int pass = 0; pass < 2; pass++) {
    float start = motor_driver.get_field_angle();
    motor_driver.rotate_field(pass == 0 ? sweep : -sweep, field_velocity, [&]() {
      MT6835Encoder::AbsRawAngleType raw = encoder.read_abs_angle_raw();
      float field = motor_driver.get_field_angle();
      if (fabsf(field - start) < settle)
        return;
      for (int k = -1; k <= 1; k++) {
        float pos = servo_controller.encoder_angle_to_motor_pos(raw + MT6835Encoder::AbsRawAngleType(k) * MT6835_CPR);
        float r = field - servo_controller.motor_pos_to_field_angle(pos);
        c[pass][k + 1] += cosf(r);
        s[pass][k + 1] += sinf(r);
      }
      n[pass]++;
    });
  }
  encoder.read_abs_angle_raw();

  for (int k = 0; k < 3; k++) {
    float r_fwd = atan2f(float(s[0][k]), float(c[0][k]));
    float r_bwd = atan2f(float(s[1][k]), float(c[1][k]));
    offset_rad[k] = wrap_angle(r_fwd + 0.5f * wrap_angle(r_bwd - r_fwd));
    if (k == 1)
      rotor_lag_rad = 0.5f * fabsf(wrap_angle(r_fwd - r_bwd));
  }

  // largo del vector medio de cada pasada: 1 si el desfase fue constante en todo el barrido
  consistency = 1.0f;
  for (int pass = 0; pass < 2; pass++) {
    float len = (n[pass] > 0) ? float(sqrt(c[pass][1] * c[pass][1] + s[pass][1] * s[pass][1]) / n[pass]) : 0.0f;
    consistency = std::min(consistency, len);
  }
}
