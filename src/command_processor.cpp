#include "command_processor.h"
#include "servo_control/cascade_corrector.h"
#include "servo_control/actuator_calibration.h"
#include "hw_config.h"
#include "utilities/logging.h"
#include "utilities/math_constants.h"
#include "hardware/sync.h"
#include "pico/time.h"

#include <Arduino.h>
#include <cstdio>
#include <algorithm>

SingleAxisCommandProcessor::SingleAxisCommandProcessor(AxisController* axis, TrapezoidalProfile* profile,
                                                       SharedData* shared_data, LinearCascadeCorrector* corrector)
  : axis(axis), profile(profile), shared_data(shared_data), cascade_corrector(corrector)
{
  max_velocity_mm_s = DEFAULT_MAX_VELOCITY_MM_S;
  max_accel_mm_s2 = DEFAULT_MAX_ACCEL_MM_S2;
}

void SingleAxisCommandProcessor::send_reply(const char* str) {
  Serial.print(str);
}

bool SingleAxisCommandProcessor::can_process_command(const GCodeCommand& cmd) {
  // Always accept: unknown commands get an explicit error from process_command()
  // instead of being retried forever.
  return true;
}

void SingleAxisCommandProcessor::process_command(const GCodeCommand& cmd, std::string& reply) {
  const std::string& c = cmd.get_command();

  if (c == "G0" || c == "G1") { process_move_command(cmd, reply); return; }
  if (c == "G28") { process_home_command(cmd, reply); return; }
  if (c == "G90") { absolute_mode = true; reply = "ok\n"; return; }
  if (c == "G91") { absolute_mode = false; reply = "ok\n"; return; }

  if (c == "M17") {
    // Without calibration commutation does not know the rotor angle and the loop runs away.
    if (!axis->is_calibrated()) { reply = "error: axis not calibrated, run M56 first\n"; return; }
    // Without homing (e.g. after boot with the calibration loaded from flash) the encoder
    // count may be off by one period relative to the table, shifting commutation by 120/240
    // electrical degrees so the motor pushes the wrong way. G28 checks and fixes that.
    if (!axis->is_homed()) { reply = "error: axis not homed, run G28 (or M56) first\n"; return; }
    // Re-arm holding the carriage where it is, so the loop does not start with the stale
    // target of an interrupted G1 and immediately trip the fault cutoff. Resend G1 after M17.
    double pos_mm = axis->get_position_mm();
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(pos_mm, pos_mm, max_velocity_mm_s, max_accel_mm_s2);
    restore_interrupts(irq_state);
    current_target_mm = pos_mm;
    shared_data->set_target_position_mm(pos_mm);
    cascade_corrector->reset();
    axis->set_motor_enabled(true);
    reply = "ok\n";
    return;
  }
  if (c == "M18") { axis->set_motor_enabled(false); reply = "ok\n"; return; }

  if (c == "M55") { process_tune_pid_command(cmd, reply); return; }
  if (c == "M56") { process_calibrate_command(cmd, reply); return; }
  if (c == "M114") { process_report_position_command(reply); return; }
  if (c == "M119") { process_report_status_command(reply); return; }
  if (c == "M120") { process_telemetry_command(cmd, reply); return; }
  if (c == "M58") {
    // M58 [V<0|1>] [F<0|1>]: V = velocity from the LUT slope (1) or raw counts (0);
    // F = profile velocity feedforward.
    ServoController* servo = axis->get_servo_controller();
    if (cmd.has_word('V')) servo->velocity_from_lut = cmd.get_value('V') >= 1.0f;
    if (cmd.has_word('F')) servo->feedforward_enabled = cmd.get_value('F') >= 1.0f;
    char buf[96];
    snprintf(buf, sizeof(buf), "velocidad_tabla=%d realimentacion_anticipada=%d\n",
             (int)servo->velocity_from_lut, (int)servo->feedforward_enabled);
    reply = std::string(buf) + "ok\n";
    return;
  }
  if (c == "M59") {
    // M59: anti-cogging compensation. Without arguments it reports the status.
    //   S<0|1> enable/disable       G<gain>          C clear tables
    //   I<idx> V<el. deg>  direction-independent part
    //   J<idx> V<el. deg>  part that flips sign with direction (scales with speed, see R)
    //   R<mm/s> reference speed of the direction-dependent part
    //   P print tables              W store to flash L load from flash
    ServoController* servo = axis->get_servo_controller();
    const int n = ServoController::COGGING_TABLE_SIZE;
    if (cmd.has_word('C')) {
      for (int i = 0; i < n; i++) servo->cogging_table[i] = servo->cogging_table_dir[i] = 0.0f;
    }
    if (cmd.has_word('I') && cmd.has_word('V')) {
      int i = int(cmd.get_value('I'));
      if (i < 0 || i >= n) { reply = "error: index out of range\n"; return; }
      servo->cogging_table[i] = cmd.get_value('V') * Constants::DEG2RAD;
    }
    if (cmd.has_word('J') && cmd.has_word('V')) {
      int i = int(cmd.get_value('J'));
      if (i < 0 || i >= n) { reply = "error: index out of range\n"; return; }
      servo->cogging_table_dir[i] = cmd.get_value('V') * Constants::DEG2RAD;
    }
    if (cmd.has_word('R') && cmd.get_value('R') > 0.0f)
      servo->cogging_dir_ref = cmd.get_value('R') / LEAD_SCREW_PITCH_MM * Constants::TWO_PI_F;
    if (cmd.has_word('G')) servo->cogging_gain = cmd.get_value('G');
    if (cmd.has_word('S')) servo->cogging_enabled = cmd.get_value('S') >= 1.0f;
    if (cmd.has_word('L') && !axis->load_cogging()) { reply = "error: no anti-cogging table in flash\n"; return; }
    if (cmd.has_word('W') && !axis->store_cogging()) { reply = "error: could not save\n"; return; }
    char buf[160];
    if (cmd.has_word('P')) {
      for (int i = 0; i < n; i++) {
        snprintf(buf, sizeof(buf), "cog %d %.4f %.4f\n", i, servo->cogging_table[i] * Constants::RAD2DEG,
                 servo->cogging_table_dir[i] * Constants::RAD2DEG);
        reply += buf;
      }
    }
    float amp = 0.0f;
    float amp_dir = 0.0f;
    for (int i = 0; i < n; i++) {
      amp = std::max(amp, fabsf(servo->cogging_table[i]));
      amp_dir = std::max(amp_dir, fabsf(servo->cogging_table_dir[i]));
    }
    snprintf(buf, sizeof(buf), "anti_cogging=%d gain=%.3f points=%d max=%.3f deg max_dir=%.3f deg "
             "ref=%.3f mm/s field_at_zero=%.6f rad\n",
             (int)servo->cogging_enabled, (float)servo->cogging_gain, n, amp * Constants::RAD2DEG,
             amp_dir * Constants::RAD2DEG, servo->cogging_dir_ref / Constants::TWO_PI_F * LEAD_SCREW_PITCH_MM,
             axis->field_angle_at_zero());
    reply += std::string(buf) + "ok\n";
    return;
  }
  if (c == "M203") {
    // M203 X<mm/s>: maximum G0/G1 velocity (default DEFAULT_MAX_VELOCITY_MM_S)
    if (cmd.has_word('X')) {
      float v = cmd.get_value('X');
      if (!(v > 0.0f && v <= MAX_VELOCITY_LIMIT_MM_S)) {
        char e[80];
        snprintf(e, sizeof(e), "error: velocity out of range (0, %.1f] mm/s\n", MAX_VELOCITY_LIMIT_MM_S);
        reply = e;
        return;
      }
      max_velocity_mm_s = v;
    }
    char buf[80];
    snprintf(buf, sizeof(buf), "max_velocity=%.3f mm/s\nok\n", max_velocity_mm_s);
    reply = buf;
    return;
  }
  if (c == "M121") { axis->dump_trace(); reply = "ok\n"; return; }
  if (c == "M57") {
    // Diagnostic: M57 X<distance mm> [F<mm/s>] [P<decimation>]. One-way open-loop sweep
    // printing the commutation offset. Leaves the motor energized; send M17 afterwards.
    if (!axis->is_homed() || !axis->is_calibrated()) { reply = "error: axis not homed/calibrated\n"; return; }
    if (!cmd.has_word('X')) { reply = "error: missing X (distance in mm)\n"; return; }
    float dist = cmd.get_value('X');
    float end_mm = axis->get_position_mm() + dist;
    if (end_mm < AXIS_MIN_POSITION_MM || end_mm > AXIS_MAX_POSITION_MM) {
      reply = "error: the sweep ends out of range\n";
      return;
    }
    float speed = std::min(cmd.get_value('F', 1.6f), 2.0f);
    int decimation = std::max(1, int(cmd.get_value('P', 4.0f)));

    shared_data->servo_loop_enabled = false;
    sleep_ms(5);
    axis->commutation_sweep(dist, speed, decimation);
    double pos_mm = axis->get_position_mm();
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(pos_mm, pos_mm, max_velocity_mm_s, max_accel_mm_s2);
    restore_interrupts(irq_state);
    current_target_mm = pos_mm;
    shared_data->set_target_position_mm(pos_mm);
    shared_data->servo_loop_enabled = true;
    reply = "ok\n";
    return;
  }

  reply = "error: unknown command '" + c + "'\n";
}

void SingleAxisCommandProcessor::process_move_command(const GCodeCommand& cmd, std::string& reply) {
  if (!axis->is_homed() || !axis->is_calibrated()) {
    reply = "error: axis not homed/calibrated, run G28 and M56 first\n";
    return;
  }

  if (!cmd.has_word('X')) {
    reply = "error: missing parameter X\n";
    return;
  }

  double x = cmd.get_value_double('X', 0.0);
  double target_mm = absolute_mode ? x : (current_target_mm + x);

  if (target_mm < AXIS_MIN_POSITION_MM || target_mm > AXIS_MAX_POSITION_MM) {
    reply = "error: position out of range [" +
            std::to_string(AXIS_MIN_POSITION_MM) + ", " +
            std::to_string(AXIS_MAX_POSITION_MM) + "] mm\n";
    return;
  }

  float feedrate = cmd.get_value('F', max_velocity_mm_s);
  feedrate = std::min(feedrate, max_velocity_mm_s);

  // The Core 0 timer ISR calls profile->update(); disable IRQs briefly so start() is not
  // interrupted halfway. The new move starts from the current profile position so a G1
  // issued during another move does not make the target jump.
  uint32_t irq_state = save_and_disable_interrupts();
  profile->start(profile->get_current_position(), target_mm, feedrate, max_accel_mm_s2);
  restore_interrupts(irq_state);

  current_target_mm = target_mm;
  reply = "ok\n";
}

//--- Anti-cogging relearn after calibration (see COGGING_RELEARN_ON_CALIBRATION) -------

static void set_cogging_tables(ServoController* servo, const float ts_deg[], const float ta_deg[]) {
  bool any = false;
  for (int i = 0; i < ServoController::COGGING_TABLE_SIZE; i++) {
    servo->cogging_table[i] = ts_deg[i] * Constants::DEG2RAD;
    servo->cogging_table_dir[i] = ta_deg[i] * Constants::DEG2RAD;
    any = any || ts_deg[i] != 0.0f || ta_deg[i] != 0.0f;
  }
  servo->cogging_gain = 1.0f;
  servo->cogging_dir_ref = COGGING_DIR_REF_MM_S / LEAD_SCREW_PITCH_MM * Constants::TWO_PI_F;
  servo->cogging_enabled = any;
}

// One leg to 'dest' at the learning speed, accumulating the error. Returns false if aborted.
static bool relearn_leg(AxisController* axis, TrapezoidalProfile* profile, SharedData* shared_data,
                        float dest, float accel, bool measure = true) {
  uint32_t irq_state = save_and_disable_interrupts();
  profile->start(profile->get_current_position(), dest, COGGING_DIR_REF_MM_S, accel);
  restore_interrupts(irq_state);
  shared_data->cogging_learner.active = measure;
  bool ok = true;
  while (!profile->is_idle()) {
    sleep_ms(5);
    double target = profile->get_current_position();
    double pos = axis->get_position_mm();
    float err_um = float(1000.0 * fabs(target - pos));
    if (axis->has_servo_fault() || err_um > COGGING_RELEARN_ABORT_UM) {
      LOG_ERROR("Anti-cogging relearn: aborted towards %.1f mm: target %.4f mm, position %.4f mm, "
                "error %.1f um, fault %d", dest, target, pos, err_um, (int)axis->has_servo_fault());
      ok = false;
      break;
    }
  }
  shared_data->cogging_learner.active = false;
  if (!ok) {
    // stop at the current profile position
    irq_state = save_and_disable_interrupts();
    double p = profile->get_current_position();
    profile->start(p, p, COGGING_DIR_REF_MM_S, accel);
    restore_interrupts(irq_state);
    return false;
  }
  sleep_ms(1000);   // dwell at the end of the leg
  return true;
}

static bool relearn_cogging(AxisController* axis, TrapezoidalProfile* profile, SharedData* shared_data,
                            float accel, bool store_to_flash, std::string& reply) {
  constexpr int N = ServoController::COGGING_TABLE_SIZE;
  static_assert(N == CoggingLearner::N, "table sizes differ");
  ServoController* servo = axis->get_servo_controller();
  float ts0[N], ta0[N], ts[N], ta[N], best_ts[N], best_ta[N];
  for (int i = 0; i < N; i++) {
    ts0[i] = ts[i] = best_ts[i] = servo->cogging_table[i] * Constants::RAD2DEG;
    ta0[i] = ta[i] = best_ta[i] = servo->cogging_table_dir[i] * Constants::RAD2DEG;
  }
  const bool was_enabled = servo->cogging_enabled;
  CoggingLearner& learner = shared_data->cogging_learner;
  learner.begin(axis->field_angle_at_zero(), MOTOR_POLE_PAIRS * Constants::TWO_PI_F / LEAD_SCREW_PITCH_MM,
                COGGING_DIR_REF_MM_S);
  // Calibration leaves the loop disabled: re-arm in place (like M17), then go to 0.
  double pos_mm = axis->get_position_mm();
  uint32_t irq_state = save_and_disable_interrupts();
  profile->start(pos_mm, pos_mm, COGGING_DIR_REF_MM_S, accel);
  restore_interrupts(irq_state);
  shared_data->set_target_position_mm(pos_mm);
  axis->set_motor_enabled(true);
  if (!relearn_leg(axis, profile, shared_data, 0.0f, accel, false)) {
    reply += "anti-cogging: could not reach 0, not relearned\n";
    return false;
  }
  // settle: 200 ms in a row below 20 um, 5 s timeout
  for (int t = 0, quiet = 0; quiet < 40; t++) {
    sleep_ms(5);
    float err_um = float(1000.0 * fabs(profile->get_current_position() - axis->get_position_mm()));
    quiet = err_um < 20.0f ? quiet + 1 : 0;
    if (t > 1000) {
      LOG_ERROR("Anti-cogging relearn: the axis does not settle at 0 (error %.1f um)", err_um);
      reply += "anti-cogging: not settled, not relearned\n";
      return false;
    }
  }

  float best_rms = 1e9f;
  int best_k = -1;
  char buf[96];
  reply += "anti-cogging, RMS per pass [um]:";
  for (int k = 0; k < COGGING_RELEARN_PASSES; k++) {
    set_cogging_tables(servo, ts, ta);
    learner.reset();
    LOG_INFO("Anti-cogging relearn: pass %i of %i", k + 1, COGGING_RELEARN_PASSES);
    bool ok = relearn_leg(axis, profile, shared_data, COGGING_RELEARN_END_MM, accel) &&
              relearn_leg(axis, profile, shared_data, 0.0f, accel);
    float e[2][N];
    if (ok && !(learner.pattern(0, e[0]) && learner.pattern(1, e[1]))) {
      LOG_ERROR("Anti-cogging relearn: too few cruise samples (RMS %.3f um)", learner.rms_um());
      ok = false;
    }
    if (!ok) {
      LOG_ERROR("Anti-cogging relearn: pass aborted, previous tables kept");
      for (int i = 0; i < N; i++) { ts[i] = ts0[i]; ta[i] = ta0[i]; }
      set_cogging_tables(servo, ts, ta);
      servo->cogging_enabled = was_enabled;
      reply += " aborted\n";
      return false;
    }
    float rms = learner.rms_um();
    LOG_INFO("Anti-cogging relearn: pass %i, RMS %.3f um", k + 1, rms);
    snprintf(buf, sizeof(buf), " %.2f", rms);
    reply += buf;
    if (rms < best_rms) {
      best_rms = rms;
      best_k = k;
      for (int i = 0; i < N; i++) { best_ts[i] = ts[i]; best_ta[i] = ta[i]; }
    }
    if (k + 1 < COGGING_RELEARN_PASSES)
      CoggingLearner::newton_step(ts, ta, e, COGGING_RELEARN_GAIN);
  }
  set_cogging_tables(servo, best_ts, best_ta);
  snprintf(buf, sizeof(buf), " -> keeping the table from pass %d\n", best_k + 1);
  reply += buf;
  LOG_INFO("Anti-cogging relearn: keeping the table from pass %i (RMS %.3f um)", best_k + 1, best_rms);
  if (store_to_flash && best_k > 0)
    axis->store_cogging();
  return true;
}

// Homing against the positive stop (D1) is not implemented: calibration always sweeps in the
// positive direction from home, so with D1 the axis would think it is at 0 at the far end.
static bool homing_direction_supported(const GCodeCommand& cmd, std::string& reply) {
  if (cmd.get_value('D', -1.0f) >= 0.0f) {
    reply = "error: homing only against the negative stop (D-1); D1 is not implemented\n";
    return false;
  }
  return true;
}

void SingleAxisCommandProcessor::process_home_command(const GCodeCommand& cmd, std::string& reply) {
  if (!homing_direction_supported(cmd, reply)) return;
  float direction_sign = -1.0f;
  bool search_stop = !cmd.has_word('H');  // G28 H: home here without a stop (motor uncoupled)

  // Core 1 must not touch the encoder/driver while homing blocks Core 0.
  shared_data->servo_loop_enabled = false;
  sleep_ms(5); // let Core 1 finish any update() in progress

  bool ok = axis->home(direction_sign, search_stop);

  if (ok) {
    current_target_mm = 0.0f;
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(0.0f, 0.0f, max_velocity_mm_s, max_accel_mm_s2); // hold the profile at 0
    restore_interrupts(irq_state);
    shared_data->set_target_position_mm(0.0);
    cascade_corrector->reset();  // clear integrators to avoid residual windup
  }

  shared_data->servo_loop_enabled = true;
  reply = ok ? "ok\n" : "error: homing fallido\n";
}

void SingleAxisCommandProcessor::process_tune_pid_command(const GCodeCommand& cmd, std::string& reply) {
  bool has_all = cmd.has_word('A') && cmd.has_word('B') && cmd.has_word('C') &&
                 cmd.has_word('D') && cmd.has_word('F');
  if (!has_all) {
    reply = "error: expected parameters A(pos_kp) B(pos_ki) C(vel_kp) D(vel_ki) F(vel_lowpass_tc)\n";
    return;
  }

  ServoController* servo = axis->get_servo_controller();
  servo->velocity_lowpass.set_time_constant(cmd.get_value('F'));
  servo->pos_controller.set_parameter(cmd.get_value('A'), cmd.get_value('B'), 0.0f,
                                      Constants::PI_F * 2.0f, Constants::PI_F * 0.5f);
  servo->velocity_controller.set_parameter(cmd.get_value('C'), cmd.get_value('D'), 0.0f,
                                           Constants::PI_F * 0.45f, Constants::PI_F * 0.45f);
  reply = "ok\n";
}

void SingleAxisCommandProcessor::process_calibrate_command(const GCodeCommand& cmd, std::string& reply) {
  bool store_to_flash = cmd.has_word('S');
  bool print_measurements = cmd.has_word('P');
  if (!homing_direction_supported(cmd, reply)) return;
  float direction_sign = -1.0f;
  bool search_stop = !cmd.has_word('H');  // M56 H: calibrate here without searching the stop
  // M56 F<0|1>: filter motor harmonics out of the table (default CALIBRATION_FILTER_MOTOR_HARMONICS)
  set_calibration_motor_filter(cmd.has_word('F') ? cmd.get_value('F') >= 1.0f : CALIBRATION_FILTER_MOTOR_HARMONICS);

  shared_data->servo_loop_enabled = false;
  sleep_ms(5);

  bool ok = axis->calibrate(direction_sign, store_to_flash, print_measurements, search_stop);

  if (ok) {
    current_target_mm = 0.0f;
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(0.0f, 0.0f, max_velocity_mm_s, max_accel_mm_s2);
    restore_interrupts(irq_state);
    shared_data->set_target_position_mm(0.0);
    cascade_corrector->reset();  // clear integrators to avoid residual windup
  }

  shared_data->servo_loop_enabled = true;

  // M56 A<0|1>: relearn the anti-cogging tables after calibrating
  bool relearn = cmd.has_word('A') ? cmd.get_value('A') >= 1.0f : (COGGING_RELEARN_ON_CALIBRATION && search_stop);
  reply.clear();
  if (ok && relearn)
    relearn_cogging(axis, profile, shared_data, max_accel_mm_s2, store_to_flash, reply);
  reply += ok ? "ok\n" : "error: calibration failed\n";
}

void SingleAxisCommandProcessor::process_report_position_command(std::string& reply) {
  // Position computed by Core 1: reading the encoder here would collide with its SPI reads.
  double pos_mm = axis->get_position_mm();
  char buf[64];
  snprintf(buf, sizeof(buf), "X:%.6f\n", pos_mm);
  reply = std::string(buf) + "ok\n";
}

void SingleAxisCommandProcessor::process_report_status_command(std::string& reply) {
  MT6835Encoder* encoder = axis->get_encoder();
  float encoder_angle_deg = encoder->get_last_abs_angle() * Constants::RAD2DEG;
  int32_t crc_errors = encoder->is_crc_enabled() ? (int32_t)encoder->get_crc_error_count(false) : -1;

  char buf[256];
  snprintf(buf, sizeof(buf),
    "homed=%d calibrated=%d target_mm=%.6f pos_mm=%.6f err_mm=%.6f enc_deg=%.3f crc_errors=%d\n",
    axis->is_homed(), axis->is_calibrated(),
    current_target_mm, axis->get_position_mm(), axis->get_position_error_mm(),
    encoder_angle_deg, (int)crc_errors);
  reply = std::string(buf);

  ServoController* servo = axis->get_servo_controller();
  auto& driver = servo->get_motor_driver();
  snprintf(buf, sizeof(buf),
    "servo: fault=%d motor_update=%d motor_pos=%.4f rad vel=%.3f rad/s out=%.3f field=%.3f amp=%.2f\n",
    axis->has_servo_fault(), servo->is_motor_update_enabled(), servo->get_position(), servo->get_velocity(),
    servo->get_output(), driver.get_field_angle(), driver.get_amplitude());
  reply += std::string(buf);

  const AxisController::FaultInfo& f = axis->get_last_fault();
  if (f.count > 0) {
    snprintf(buf, sizeof(buf),
      "last fault (%lu in total): target_mm=%.4f pos_mm=%.4f err_mm=%.4f vel=%.3f rad/s out=%.3f status=%u\n",
      (unsigned long)f.count, f.target_mm, f.pos_mm, f.target_mm - f.pos_mm, f.vel, f.out, (unsigned)f.status);
    reply += std::string(buf);
  }

  reply += "ok\n";
}

void SingleAxisCommandProcessor::process_telemetry_command(const GCodeCommand& cmd, std::string& reply) {
  if (!cmd.has_word('S')) {
    reply = "error: M120 requires parameter S (1=on, 0=off)\n";
    return;
  }

  bool enable = cmd.get_value('S') >= 1.0f;
  shared_data->telemetry_decimation = std::max(1, int(cmd.get_value('P', 1.0f)));
  shared_data->telemetry_enabled = enable;

  if (enable) {
    LOG_INFO("Telemetria binaria activada");
  } else {
    LOG_INFO("Telemetria binaria desactivada");
  }

  reply = "ok\n";
}
