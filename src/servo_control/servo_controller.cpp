// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "hardware/timer.h"
#include "pico/stdlib.h"

#include "servo_controller.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"

#include "hw_config.h"

#include <algorithm>

ServoController::ServoController(
  MOTOR_DRIVER_TYPE& motor_driver, 
  ENCODER_TYPE& encoder, 
  int32_t motor_pole_pair_count) :
    motor_driver(motor_driver),
    encoder(encoder)
{
  ServoController::motor_pole_pair_count = motor_pole_pair_count;
  ServoController::motor_update_enabled = false;
  ServoController::encoder_update_enabled = true;
  ServoController::motor_pos = 0.0f;
  ServoController::pos_error = 0.0f;

  // default encoder LUT until calibrated: one motor revolution = ENCODER_PERIODS_PER_REV
  // encoder periods
  using namespace Constants;
  float g = float(encoder.get_rawcounts_per_rev())*float(ENCODER_PERIODS_PER_REV);
  build_linear_lut(encoder_raw_to_motor_pos_lut, -g, g, -TWO_PI_F, TWO_PI_F);
}

void ServoController::init(float max_motor_amplitude) {
  ServoController::motor_current_amplitude = max_motor_amplitude;

  // setup motor driver
  motor_driver.begin();
  motor_driver.set_amplitude(0.0f, true); // correct amplitude will be set by 'set_motor_enabled()' 
  motor_driver.enable();
  motor_driver.set_field_angle(0.0f);

  velocity_lowpass.set_time_constant(VEL_LOWPASS_TC);
  velocity_from_lut = VELOCITY_FROM_LUT;
  feedforward_enabled = VELOCITY_FEEDFORWARD;
  cogging_dir_ref = COGGING_DIR_REF_MM_S / LEAD_SCREW_PITCH_MM * Constants::TWO_PI_F;

  pos_controller.set_parameter(POS_KP, POS_KI, 0.0f, Constants::PI_F*2.0F, Constants::PI_F*0.5F);
  velocity_controller.set_parameter(VEL_KP, VEL_KI, 0.0f, Constants::PI_F*0.45f, Constants::PI_F*0.45f);
}

void ServoController::set_enc_to_pos_lut(LookupTable& lut) {
  ServoController::encoder_raw_to_motor_pos_lut = lut;
  lut_periods = INT32_MIN;   // recompute the period offset with the new table
}

// get the encoder raw angle to motor position lookup table
const LookupTable& ServoController::get_enc_to_pos_lut() const {
  return encoder_raw_to_motor_pos_lut;
}

void ServoController::set_pos_to_field_lut(LookupTable& lut) {
  ServoController::motor_pos_to_field_angle_lut = lut;
}

// get the motor position to field angle lookup table
const LookupTable& ServoController::get_pos_to_field_lut() const {
  return motor_pos_to_field_angle_lut;
}


// Local slope [motor rad per count] of the encoder -> motor table at 'raw', i.e. the slope of the
// table segment containing it. With a periodic table, 'raw' is first wrapped into the
// revolution covered by the table.
static float lut_slope(const LookupTable& lut, ServoController::ENCODER_TYPE::AbsRawAngleType raw) {
  int32_t n = lut.size();
  float in_min, in_max;
  lut.get_intput_range(in_min, in_max);
  if (n < 2 || in_max <= in_min)
    return 0.0f;
  float x = float(raw);
  float period = lut.get_period_in();
  if (period > 0.0f)
    x -= floorf((x - in_min) / period) * period;
  float cell = (in_max - in_min) / float(n - 1);
  int32_t i = std::clamp(int32_t(floorf((x - in_min) / cell)), int32_t(0), n - 2);
  return (lut.get_entry(i + 1) - lut.get_entry(i)) / cell;
}

void ServoController::update(double target_motor_pos, float dt, float one_over_dt, float velocity_ff) {
  if(encoder_update_enabled == false)
    return;

  // read encoder
  ENCODER_TYPE::AbsRawAngleType encoder_angle_raw = encoder.read_abs_angle_raw();

  // convert encoder angle to motor pos using LUT and compute field angle
  int32_t periods;
  float local_pos = encoder_raw_to_motor_pos_lut.evaluate_periodic_split(encoder_angle_raw, periods);
  if (periods != lut_periods) {
    lut_periods = periods;
    lut_periods_offset = double(periods) * double(encoder_raw_to_motor_pos_lut.get_period_out());
  }
  motor_pos = lut_periods_offset + double(local_pos);
  // float is sufficient for commutation (0.002 electrical rad at 320 motor rad)
  float field_angle = motor_pos_to_field_angle(float(motor_pos));

  // subtract in double; the small error then fits in a float without loss
  pos_error = float(target_motor_pos - motor_pos);

  // velocity estimate
  // With a periodic table, velocity comes straight from the (integer) encoder counts, avoiding
  // the table seam and the float resolution loss at hundreds of rad.
  float velocity_unfiltered;
  float period_in = encoder_raw_to_motor_pos_lut.get_period_in();
  if (period_in > 0.0f) {
    // By default the counts are scaled by the local table slope. With a constant factor the raw
    // counts carry the encoder non-linearity (a few um per magnet), so the velocity estimate
    // ripples at constant speed and the loop would "correct" it, rippling the real motion.
    float rad_per_count = velocity_from_lut
      ? lut_slope(encoder_raw_to_motor_pos_lut, encoder_angle_raw)
      : encoder_raw_to_motor_pos_lut.get_period_out() / period_in;
    velocity_unfiltered = float(encoder_angle_raw - encoder_angle_raw_prev) * rad_per_count * one_over_dt;
  } else {
    velocity_unfiltered = float(motor_pos - motor_pos_prev)*one_over_dt;
  }
  encoder_angle_raw_prev = encoder_angle_raw;
  velocity = velocity_lowpass.update(velocity_unfiltered, dt);
  motor_pos_prev = motor_pos;

  // With the loop disabled (open loop, fault cut-off) the PIDs are held at zero here on Core 1,
  // so they never integrate while disabled and Core 0 never has to reset them concurrently with
  // update(); otherwise the position integrator could start out saturated.
  if(!motor_update_enabled) {
    pos_controller.reset();
    velocity_controller.reset();
    output = 0.0f;
    return;
  }

  // position control loop
  float velocity_target = pos_controller.compute(pos_error, dt, one_over_dt);
  if (feedforward_enabled)
    velocity_target += velocity_ff;

  // velocity control loop
  float torque_target = velocity_controller.compute(velocity_target-velocity, dt, one_over_dt);

  // torque control loop
  output = torque_target;

  // anti-cogging: added to the field lead, with the same limit
  float lead = output;
  if (cogging_enabled)
    lead = std::clamp(output + cogging_feedforward(field_angle, velocity_ff), -Constants::PI_F * 0.45f,
                      Constants::PI_F * 0.45f);

  // set new field direction
  motor_driver.set_field_angle(field_angle + lead);
}

float ServoController::cogging_feedforward(float field_angle, float velocity_ff) const {
  float ph = field_angle * (1.0f / Constants::TWO_PI_F);
  ph -= floorf(ph);
  float fi = ph * COGGING_TABLE_SIZE;
  int i = int(fi);
  if (i >= COGGING_TABLE_SIZE) i = COGGING_TABLE_SIZE - 1;
  float w = fi - float(i);
  int j = (i + 1) % COGGING_TABLE_SIZE;
  float d = std::clamp(velocity_ff / cogging_dir_ref, -1.0f, 1.0f);
  // fades out above the learning velocity (see COGGING_FADE_END)
  float over = fabsf(velocity_ff) / cogging_dir_ref - 1.0f;
  float fade = over <= 0.0f ? 1.0f : std::max(0.0f, 1.0f - over / (COGGING_FADE_END - 1.0f));
  float sym = cogging_table[i] * (1.0f - w) + cogging_table[j] * w;
  float dir = cogging_table_dir[i] * (1.0f - w) + cogging_table_dir[j] * w;
  return cogging_gain * fade * (sym + d * dir);
}

void ServoController::resync() {
  encoder_angle_raw_prev = encoder.read_abs_angle_raw();
  motor_pos = encoder_angle_to_motor_pos(encoder_angle_raw_prev);
  motor_pos_prev = motor_pos;
  pos_error = 0.0f;
  output = 0.0f;
  pos_controller.reset();
  velocity_controller.reset();
  velocity_lowpass.reset(0.0f);
}

bool ServoController::at_position(float motor_pos_eps) {
  return fabs(pos_error) < motor_pos_eps;
}

double ServoController::read_position() {
  return encoder_angle_to_motor_pos(encoder.read_abs_angle_raw());
}

double ServoController::get_position() {
  return motor_pos;
}

float ServoController::get_position_error() {
  return pos_error;
}

bool ServoController::move_to(float target_motor_pos, float at_pos_eps, float settle_time_s, float timeout_s) {
  uint64_t start_time_us = time_us_64();
  uint64_t time_us = start_time_us;
  uint64_t pos_reached_time_us = 0;
  uint64_t last_time = time_us;
  uint32_t settle_time_us = settle_time_s*1e6f;
  uint32_t timeout_us = timeout_s*1e6f;

  do {
    // get time and delta time
    time_us = time_us_64();
    float dt = float(time_us - last_time)*1e-6f;
    last_time = time_us;

    float pos_error;
    update(target_motor_pos, dt, 1.0f/dt);

    // check if target position reached
    if(pos_reached_time_us == 0) {
      if(at_position(at_pos_eps))
        pos_reached_time_us = time_us;
    } else {
      if(time_us-pos_reached_time_us > settle_time_us)
        return true;
    }

  } while(time_us-start_time_us < timeout_us);

  return false;
}

void ServoController::move_to_open_loop(float delta_motor_pos, float motor_angular_velocity) {
  // Determine direction of movement at the start
  const bool moving_forward = delta_motor_pos > 0.0f;

  uint64_t last_time = time_us_64();
  float pos = 0.0f;
  while (fabs(pos) < delta_motor_pos)
  {
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;

    // update encoder regularly
    if(encoder_update_enabled)
      encoder.read_abs_angle_raw();

    // update motor position
    pos += moving_forward ? motor_angular_velocity * dt : -motor_angular_velocity * dt;

    // set field angle to new position
    float clamped_motor_pos = moving_forward ? std::min(pos, delta_motor_pos) : 
                                               std::max(pos, -delta_motor_pos);
    motor_driver.set_field_angle(clamped_motor_pos*motor_pole_pair_count);
    sleep_us(100);
  }

  motor_pos += delta_motor_pos;
}

ServoController::ENCODER_TYPE& ServoController::get_encoder() {
  return encoder;
}

ServoController::MOTOR_DRIVER_TYPE& ServoController::get_motor_driver() {
  return motor_driver;
}

float ServoController::get_pole_pair_count() {
  return motor_pole_pair_count;
}

void ServoController::set_motor_enabled(bool enable, bool synchronize_field_angle) {
  if(enable) {
    // synchronize field angle to motor_pos
    if(synchronize_field_angle) {
      float start_field_angle = motor_pos_to_field_angle(float(motor_pos));
      motor_driver.set_field_angle(start_field_angle);
    }

    motor_driver.set_amplitude_smooth(motor_current_amplitude, 100);
    pos_controller.reset();
    velocity_controller.reset();
    velocity_lowpass.reset(0.0f);
    motor_pos_prev = motor_pos;

  } else {
    motor_driver.set_amplitude_smooth(0.0f, 100);
  }
}

// enable or disable servo loop update. Only sets the flag: update() holds the PIDs at zero
// while disabled, so this can be called from the other core.
void ServoController::set_motor_update_enabled(bool enable) {
  motor_update_enabled = enable;
}

void ServoController::set_encoder_update_enabled(bool enable) {
  pos_controller.reset();
  velocity_controller.reset();
  motor_pos_prev = motor_pos;
  encoder_update_enabled = enable;
}

double ServoController::encoder_angle_to_motor_pos(ENCODER_TYPE::AbsRawAngleType encoder_angle_raw) {
  return encoder_raw_to_motor_pos_lut.evaluate_periodic(encoder_angle_raw);
}

float ServoController::motor_pos_to_field_angle(float motor_pos) {
  return motor_pos_to_field_angle_lut.evaluate(motor_pos);
}