#include <LittleFS.h>
#include <algorithm>
#include <cmath>
#include "pico/multicore.h"

#include "axis_controller.h"
#include "hw_config.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"
#include "utilities/lookup_table.h"
#include "servo_control/homing_controller.h"
#include "servo_control/actuator_calibration.h"

AxisController::AxisController(MT6835Encoder* encoder, TB6612MotorDriver* motor_driver,
                               float motor_pole_pairs, float lead_screw_pitch_mm)
  : encoder(encoder), motor_driver(motor_driver), lead_screw_pitch_mm(lead_screw_pitch_mm)
{
  // Same 2*pi constant as motor_pos_rad_to_mm() and the LUT period (period_out)
  mm_to_rad = double(Constants::TWO_PI_F) / double(lead_screw_pitch_mm);
  rad_to_q = 1.0 / (mm_to_rad * POSITION_LSB_MM);
  servo_controller = new ServoController(*motor_driver, *encoder, (int32_t)motor_pole_pairs);
}

AxisController::~AxisController() {
  delete servo_controller;
  delete motor_driver;
  delete encoder;
}

void AxisController::init() {
  if (!encoder->init(0x5, 0x4)) {
    LOG_ERROR("AxisController: encoder initialization failed");
  }
  encoder->set_crc_enabled(ENABLE_ENCODER_CRC);

  servo_controller->init(MOTOR_MAX_CURRENT_FACTOR);
  servo_controller->set_motor_enabled(false, false);

  load_calibration();
  if (load_cogging()) {
    servo_controller->cogging_enabled = COGGING_ENABLED_AT_BOOT;
    LOG_INFO("AxisController: anti-cogging table loaded from flash (%s)",
             COGGING_ENABLED_AT_BOOT ? "enabled" : "disabled, M59 S1 to enable it");
  }
}

bool AxisController::home(float direction_sign, bool search_stop) {
  // Homing drives the field directly, so the closed loop stays off afterwards (M17 re-enables it)
  servo_controller->set_motor_update_enabled(false);
  homed = false;

  if (search_stop) {
    LOG_INFO("AxisController: homing...");

    HomingController homing_controller;
    bool ok = homing_controller.run_blocking(
      servo_controller,
      direction_sign * HOMING_VELOCITY,
      mm_to_motor_pos_rad(HOMING_SEARCH_RANGE_MM),
      HOMING_CURRENT,
      ENCODER_ANGLE_TO_ROTOR_ANGLE,
      0.0f
    );

    if (!ok) {
      LOG_ERROR("AxisController: homing failed (hard stop not detected)");
      servo_controller->resync();
      return false;
    }
  } else {
    LOG_WARNING("AxisController: home set at the current position WITHOUT searching for the stop. Only for "
                "tests with the motor decoupled: with the carriage attached, 0..255 mm is not the real travel.");
    servo_controller->set_motor_enabled(true, false);  // energize with the current field so the rotor aligns
    sleep_ms(200);
    encoder->read_abs_angle_raw();
    encoder->reset_abs_angle_period();
  }

  // Homing fixes the encoder counter only modulo one period: enough for position, not for
  // commutation (see COMMUTATION_CHECK_CYCLES in hw_config.h).
  if (calibrated && !verify_commutation(-direction_sign, true)) {
    LOG_ERROR("AxisController: commutation does not match the stored calibration. Run M56.");
    servo_controller->set_motor_enabled(false, false);
    servo_controller->resync();
    return false;
  }

  servo_controller->resync();
  publish_position();
  homed = true;
  LOG_INFO("AxisController: homing OK (pos = %.4f mm). Closed loop off: M17 to enable it.",
           get_position_mm());
  return true;
}

bool AxisController::verify_commutation(float away_from_stop_sign, bool allow_period_correction) {
  float offset[3], lag, consistency;
  measure_commutation_offsets(*servo_controller, away_from_stop_sign, COMMUTATION_CHECK_CYCLES,
                              CALIBRATION_FIELD_VELOCITY, offset, lag, consistency);

  // offset[1]: counter as is; offset[0] / offset[2]: counter shifted by -1 / +1 period
  int best = 1;
  if (allow_period_correction) {
    for (int i = 0; i < 3; i++)
      if (fabsf(offset[i]) < fabsf(offset[best])) best = i;
  }

  LOG_INFO("Commutation: offset %.1f electrical deg (with the encoder shifted -1 period: %.1f, +1: %.1f), "
           "rotor lag %.1f deg, consistency %.2f",
           offset[1] * Constants::RAD2DEG, offset[0] * Constants::RAD2DEG, offset[2] * Constants::RAD2DEG,
           lag * Constants::RAD2DEG, consistency);

  if (consistency < 0.7f) {
    LOG_ERROR("Commutation: the offset changes during the sweep (consistency %.2f): the encoder does not "
              "follow the rotor (loose wheel, weak field, rotor blocked?)", consistency);
    return false;
  }
  if (fabsf(offset[best]) > COMMUTATION_MAX_OFFSET_DEG * Constants::DEG2RAD) {
    LOG_ERROR("Commutation: offset of %.1f electrical deg, the maximum accepted is %.1f",
              offset[best] * Constants::RAD2DEG, COMMUTATION_MAX_OFFSET_DEG);
    return false;
  }

  int k = best - 1;
  if (k != 0) {
    encoder->read_abs_angle_raw();
    encoder->reset_abs_angle(encoder->get_last_abs_raw_angle() + MT6835Encoder::AbsRawAngleType(k) * MT6835_CPR);
    LOG_WARNING("Commutation: the encoder counter was off by %d period(s) (offset of %.1f electrical "
                "deg). Corrected: now %.1f deg.", k, offset[1] * Constants::RAD2DEG,
                offset[best] * Constants::RAD2DEG);
  }
  return true;
}

bool AxisController::calibrate(float homing_direction_sign, bool store_to_flash, bool print_measurements,
                               bool search_stop) {
  // The new table is built on this homing's reference, so don't verify against the old one
  calibrated = false;
  if (!home(homing_direction_sign, search_stop)) {
    return false;
  }

  LookupTable encoder_raw_to_motor_pos_lut;
  LookupTable motor_pos_to_field_angle_lut;

  bool ok = measure_calibration_data(
    encoder_raw_to_motor_pos_lut,
    motor_pos_to_field_angle_lut,
    *servo_controller,
    CALIBRATION_RANGE * Constants::DEG2RAD,
    CALIBRATION_FIELD_VELOCITY,
    ENCODER_LUT_SIZE,
    print_measurements
  );

  if (!ok) {
    LOG_ERROR("AxisController: calibration failed");
    calibrated = false;
    return false;
  }

  servo_controller->set_enc_to_pos_lut(encoder_raw_to_motor_pos_lut);
  servo_controller->set_pos_to_field_lut(motor_pos_to_field_angle_lut);

  // Self-check: with the freshly measured table the offset must be ~0 without any correction.
  if (!verify_commutation(-homing_direction_sign, false)) {
    LOG_ERROR("AxisController: the new table does not commutate correctly. Calibration discarded.");
    servo_controller->resync();
    return false;
  }
  servo_controller->resync();
  publish_position();
  calibrated = true;

  if (store_to_flash) {
    store_calibration();
  }

  LOG_INFO("AxisController: calibration OK");
  return true;
}

void AxisController::update_servo(double target_position_mm, float dt, float one_over_dt,
                                  float target_velocity_mm_s) {
  double target_motor_pos_rad = target_position_mm * mm_to_rad;
  float velocity_ff = target_velocity_mm_s / lead_screw_pitch_mm * Constants::TWO_PI_F;
  servo_controller->update(target_motor_pos_rad, dt, one_over_dt, velocity_ff);
  publish_position();

  // Safety trip: a large error sustained for long means the encoder probably no longer follows
  // the rotor (magnet wheel slipping, weak-field zone), so commutation is misaligned and the
  // motor would just push without moving.
  constexpr float FAULT_ERROR_MM = 1.0f;
  constexpr uint64_t FAULT_TIME_US = 500000;
  if (!servo_controller->is_motor_update_enabled()) {
    fault_since_us = 0;
    return;
  }
  uint64_t now_us = time_us_64();

  // The trace freezes only on a fault trip, so it holds the ~2 s leading up to it.
  loops_since_trace++;
  if (!trace_frozen && now_us - last_trace_us >= 1000) {
    TracePoint& p = trace[trace_head];
    p.t_us = uint32_t(now_us);
    p.target_mm = float(target_position_mm);
    p.pos_mm = float(get_position_mm());
    p.vel = servo_controller->get_velocity();
    p.out = servo_controller->get_output();
    p.chip_raw = encoder->get_last_chip_raw_angle();
    p.status = encoder->get_status();
    p.loops = loops_since_trace;
    loops_since_trace = 0;
    last_trace_us = now_us;
    trace_head = (trace_head + 1) % TRACE_SIZE;
    if (trace_count < TRACE_SIZE) trace_count = trace_count + 1;
  }

  if (fabsf(get_position_error_mm()) < FAULT_ERROR_MM) {   // float is fine: the error is small
    fault_since_us = 0;
  } else if (fault_since_us == 0) {
    fault_since_us = now_us;
  } else if (now_us - fault_since_us > FAULT_TIME_US) {
    last_fault.count++;
    last_fault.target_mm = float(target_position_mm);
    last_fault.pos_mm = float(get_position_mm());
    last_fault.vel = servo_controller->get_velocity();
    last_fault.out = servo_controller->get_output();
    last_fault.status = encoder->get_status();
    servo_fault = true;
    trace_frozen = true;
    servo_controller->set_motor_update_enabled(false);
    motor_driver->set_amplitude(0.0f, true);
    fault_since_us = 0;
  }
}

void AxisController::dump_trace() {
  bool was_frozen = trace_frozen;
  trace_frozen = true;  // keep Core 1 from writing while printing
  int count = trace_count;
  int start = (trace_head - count + TRACE_SIZE) % TRACE_SIZE;
  Serial.println("t_us,target_mm,pos_mm,vel,out,chip_raw,status,loops");
  for (int i = 0; i < count; i++) {
    const TracePoint& p = trace[(start + i) % TRACE_SIZE];
    Serial.printf("%lu,%.5f,%.5f,%.3f,%.3f,%ld,%u,%u\n", (unsigned long)p.t_us, p.target_mm, p.pos_mm,
                  p.vel, p.out, (long)p.chip_raw, (unsigned)p.status, (unsigned)p.loops);
  }
  trace_frozen = was_frozen;
}

void AxisController::commutation_sweep(float distance_mm, float speed_mm_s, int decimation) {
  servo_controller->set_motor_update_enabled(false);
  servo_controller->resync();
  // energize with the field where the table places the rotor (same as M17)
  servo_controller->set_motor_enabled(true, true);
  sleep_ms(200);

  const float pole_pairs = servo_controller->get_pole_pair_count();
  const float field_velocity = fabsf(mm_to_motor_pos_rad(speed_mm_s)) * pole_pairs;
  const float sweep = mm_to_motor_pos_rad(distance_mm) * pole_pairs;
  const int dir = distance_mm >= 0.0f ? 1 : -1;
  int step = 0;

  Serial.println("dir,pos_mm,offset_deg,status");
  motor_driver->rotate_field(sweep, field_velocity, [&]() {
    MT6835Encoder::AbsRawAngleType raw = encoder->read_abs_angle_raw();
    if ((step++ % decimation) != 0)
      return;
    float pos = servo_controller->encoder_angle_to_motor_pos(raw);
    float r = remainderf(motor_driver->get_field_angle() - servo_controller->motor_pos_to_field_angle(pos),
                         Constants::TWO_PI_F);
    Serial.printf("%d,%.4f,%.2f,%u\n", dir, motor_pos_rad_to_mm(pos), r * Constants::RAD2DEG,
                  (unsigned)encoder->get_status());
  });
  Serial.println("end");

  servo_controller->resync();
  publish_position();
}

void AxisController::set_motor_enabled(bool enable) {
  // Take the field away from Core 1 first and let its in-flight update() finish (~45 us), so the
  // current ramp (Core 0) and commutation (Core 1) never write the PWM at the same time.
  servo_controller->set_motor_update_enabled(false);
  busy_wait_us(500);

  if (enable) {
    servo_fault = false;
    trace_head = 0;
    trace_count = 0;
    trace_frozen = false;
  }
  servo_controller->set_motor_enabled(enable, true);
  if (enable)
    servo_controller->set_motor_update_enabled(true);
}

double AxisController::get_position_mm() const {
  return double(position_q) * POSITION_LSB_MM;
}

void AxisController::publish_position() {
  // Runs every loop iteration: a single double multiply (software FP, ~0.8 us)
  const double q = servo_controller->get_position() * rad_to_q;
  const float qf = float(q);   // out of range (+-536 mm) only before homing
  position_q = fabsf(qf) < 2.1e9f ? int32_t(lround(q)) : (qf > 0.0f ? INT32_MAX : -INT32_MAX);
}

float AxisController::get_position_error_mm() const {
  // The conversion is purely proportional, so no offset needs subtracting
  return motor_pos_rad_to_mm(servo_controller->get_position_error());
}

bool AxisController::is_homed() const {
  return homed;
}

bool AxisController::is_calibrated() const {
  return calibrated;
}

ServoController* AxisController::get_servo_controller() {
  return servo_controller;
}

MT6835Encoder* AxisController::get_encoder() {
  return encoder;
}

// --- mm <-> rad conversions from the lead screw pitch -------------------------------
// One motor revolution (2*PI rad) moves the carriage 'lead_screw_pitch_mm' [mm].

float AxisController::mm_to_motor_pos_rad(float pos_mm) const {
  return (pos_mm / lead_screw_pitch_mm) * Constants::TWO_PI_F;
}

float AxisController::motor_pos_rad_to_mm(float motor_pos_rad) const {
  return (motor_pos_rad / Constants::TWO_PI_F) * lead_screw_pitch_mm;
}

// --- calibration persistence in flash (LittleFS) ----------------------------------

std::string AxisController::enc_to_pos_lut_filename() const {
  return "axis_enc_to_pos_lut.dat";
}

std::string AxisController::pos_to_field_lut_filename() const {
  return "axis_pos_to_field_lut.dat";
}

bool AxisController::load_calibration() {
  std::string fn1 = enc_to_pos_lut_filename();
  std::string fn2 = pos_to_field_lut_filename();

  if (!LittleFS.exists(fn1.c_str()) || !LittleFS.exists(fn2.c_str())) {
    LOG_WARNING("AxisController: no stored calibration found. Run M56.");
    return false;
  }

  LookupTable enc_to_pos_lut;
  LookupTable pos_to_field_lut;
  bool ok = true;
  ok &= load_lut_from_file(enc_to_pos_lut, fn1.c_str());
  ok &= load_lut_from_file(pos_to_field_lut, fn2.c_str());
  if (!ok) return false;

  servo_controller->set_enc_to_pos_lut(enc_to_pos_lut);
  servo_controller->set_pos_to_field_lut(pos_to_field_lut);
  calibrated = true;

  LOG_INFO("AxisController: calibration loaded from flash (size=%i,%i)",
           enc_to_pos_lut.size(), pos_to_field_lut.size());
  return true;
}

bool AxisController::store_calibration() {
  // Core 1 cannot execute from flash while it is erased/written. LittleFS pauses it with
  // rp2040.idleOtherCore(), which only works when Core 1 runs Arduino's setup1()/loop1(); here it
  // is started with multicore_launch_core1(), so use the SDK lockout instead (Core 1 calls
  // multicore_lockout_victim_init() at startup, see main.cpp).
  if (!multicore_lockout_start_timeout_us(200000)) {
    LOG_ERROR("AxisController: could not pause core 1, calibration NOT saved to flash");
    return false;
  }
  bool ok = true;
  ok &= save_lut_to_file(servo_controller->get_enc_to_pos_lut(), enc_to_pos_lut_filename().c_str());
  ok &= save_lut_to_file(servo_controller->get_pos_to_field_lut(), pos_to_field_lut_filename().c_str());
  multicore_lockout_end_blocking();
  if (ok) LOG_INFO("AxisController: calibration saved to flash");
  else LOG_ERROR("AxisController: error saving the calibration to flash");
  return ok;
}

// --- anti-cogging compensation table (M59) ---------------------------

static const char* COGGING_FILE = "axis_cogging.dat";
static constexpr uint32_t COGGING_MAGIC = 0x434F4732;   // "COG2": table, per-direction table, reference velocity

float AxisController::field_angle_at_zero() {
  return servo_controller->motor_pos_to_field_angle(0.0f);
}

bool AxisController::store_cogging() {
  // Same precaution as store_calibration(): Core 1 must not run from flash while it is written
  if (!multicore_lockout_start_timeout_us(200000)) {
    LOG_ERROR("AxisController: could not pause core 1, anti-cogging table NOT saved");
    return false;
  }
  bool ok = false;
  File f = LittleFS.open(COGGING_FILE, "w");
  if (f) {
    uint32_t n = ServoController::COGGING_TABLE_SIZE;
    float ref = servo_controller->cogging_dir_ref;
    ok = f.write((const uint8_t*)&COGGING_MAGIC, 4) == 4 && f.write((const uint8_t*)&n, 4) == 4 &&
         f.write((const uint8_t*)servo_controller->cogging_table, n * sizeof(float)) == n * sizeof(float) &&
         f.write((const uint8_t*)servo_controller->cogging_table_dir, n * sizeof(float)) == n * sizeof(float) &&
         f.write((const uint8_t*)&ref, 4) == 4;
    f.close();
  }
  multicore_lockout_end_blocking();
  if (ok) LOG_INFO("AxisController: anti-cogging table saved to flash");
  else LOG_ERROR("AxisController: error saving the anti-cogging table");
  return ok;
}

bool AxisController::load_cogging() {
  if (!LittleFS.exists(COGGING_FILE))
    return false;
  File f = LittleFS.open(COGGING_FILE, "r");
  if (!f)
    return false;
  uint32_t magic = 0, n = 0;
  float tmp[ServoController::COGGING_TABLE_SIZE], tmp_dir[ServoController::COGGING_TABLE_SIZE], ref = 0.0f;
  bool ok = f.read((uint8_t*)&magic, 4) == 4 && f.read((uint8_t*)&n, 4) == 4 && magic == COGGING_MAGIC &&
            n == ServoController::COGGING_TABLE_SIZE &&
            f.read((uint8_t*)tmp, n * sizeof(float)) == n * sizeof(float) &&
            f.read((uint8_t*)tmp_dir, n * sizeof(float)) == n * sizeof(float) && f.read((uint8_t*)&ref, 4) == 4 &&
            ref > 0.0f;
  f.close();
  if (!ok) {
    LOG_WARNING("AxisController: invalid anti-cogging table file");
    return false;
  }
  for (uint32_t i = 0; i < n; i++) {
    servo_controller->cogging_table[i] = tmp[i];
    servo_controller->cogging_table_dir[i] = tmp_dir[i];
  }
  servo_controller->cogging_dir_ref = ref;
  return true;
}
