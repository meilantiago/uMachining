// --------------------------------------------------------------------------------------
// Single-axis (X) closed-loop firmware for micromachining, adapted from MicroManipulatorStepper
// (0x23), firmware/MotionControllerRP. See "Relation to the upstream firmware" in README.md.
//
// Architecture (same idea as the original, one joint and no kinematics):
//   Core 0: parses G-code over serial; a 2 kHz timer advances the trapezoidal profile and
//           writes the target position to SharedData.
//   Core 1: free-running loop that reads SharedData and runs the servo loop
//           (encoder -> cascaded PID -> FOC commutation) as fast as possible.
// --------------------------------------------------------------------------------------

#include <Arduino.h>
#include <LittleFS.h>
#include <algorithm>

#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/vreg.h"
#include "hardware/spi.h"
#include "pico/multicore.h"

#include "hw_config.h"
#include "utilities/logging.h"

#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "axis_controller.h"
#include "motion_profile/trapezoidal_profile.h"
#include "servo_control/cascade_corrector.h"
#include "shared_data.h"
#include "command_processor.h"
#include "command_parser/command_parser.h"

//*** GLOBALS ***************************************************************************

constexpr int SPINLOCK_ID_SHARED_DATA = 0;

AxisController* axis = nullptr;
TrapezoidalProfile motion_profile;
LinearCascadeCorrector cascade_corrector;
SharedData shared_data(SPINLOCK_ID_SHARED_DATA);

CommandParser command_parser;
SingleAxisCommandProcessor* command_processor = nullptr;

struct repeating_timer motion_profile_timer;
uint64_t last_profile_update_time_us = 0;

// There is a single MT6835 encoder facing a multipole magnet wheel, so the "linear" position
// is derived from the axis controller (motor position).
double read_linear_encoder() {
  return axis->get_position_mm();
}

//*** FUNCTIONS *************************************************************************

void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  busy_wait_us(10 * 1000);
  set_sys_clock_khz(250000, true);
}

// Hardware timer callback every 500 us (2 kHz): advances the trapezoidal profile and
// publishes the new target position for Core 1.
bool update_motion_profile_isr(repeating_timer_t* timer) {
  uint64_t time_us = time_us_64();
  float dt = float(time_us - last_profile_update_time_us) * 1e-6f;
  last_profile_update_time_us = time_us;

  double pos_mm;
  float vel_mm_s;
  motion_profile.update(dt, pos_mm, vel_mm_s);

  // --- Outer cascade loop (linear encoder) ---
  // PI correction from the linear position. Without a linear encoder, linear_pos == motor position.
  double linear_pos = read_linear_encoder();
  if (shared_data.cogging_learner.active)
    shared_data.cogging_learner.add(pos_mm, linear_pos, vel_mm_s);
  double corrected_pos = pos_mm;
  if (shared_data.cascade_corrector_enabled) {
    corrected_pos = cascade_corrector.update(pos_mm, linear_pos, dt);  // computed in float: loses nm resolution
  }

  if (spin_try_lock_unsafe(shared_data.lock)) {
    shared_data.target_position_mm = corrected_pos;
    shared_data.target_velocity_mm_s = vel_mm_s;
    spin_unlock_unsafe(shared_data.lock);
  }

  // --- Telemetry ---
  // try_push is non-blocking: the point is silently dropped if the lock is taken or the buffer
  // is full. Everything read here is cached state written by Core 1 (no SPI access in the ISR).
  static uint32_t telemetry_tick = 0;
  static uint16_t telemetry_seq = 0;
  if (shared_data.telemetry_enabled && ++telemetry_tick >= shared_data.telemetry_decimation) {
    telemetry_tick = 0;
    ServoController* servo = axis->get_servo_controller();
    TelemetryPoint tp;
    tp.timestamp_us = static_cast<uint32_t>(time_us);
    tp.target_q     = int32_t(lround(pos_mm * (1.0 / POSITION_LSB_MM)));  // raw profile target, before cascade correction
    tp.motor_pos_q  = axis->get_position_q();            // = linear_pos, without a double round-trip
    tp.vel          = servo->get_velocity();
    tp.out          = servo->get_output();
    tp.chip_raw     = axis->get_encoder()->get_last_chip_raw_angle();
    tp.status       = axis->get_encoder()->get_status();
    tp.flags        = (servo->is_motor_update_enabled() ? 1 : 0) | (axis->has_servo_fault() ? 2 : 0);
    tp.seq          = telemetry_seq++;
    shared_data.telemetry_buffer.try_push(tp, shared_data.lock);
  }

  return true; // keep repeating
}

// Core 1: dedicated servo loop (encoder + cascaded PID + FOC commutation).
void main_core1() {
  LOG_INFO("Core 1: starting servo loop...");
  // Lets Core 0 pause this core while writing flash (see AxisController::store_calibration)
  multicore_lockout_victim_init();

  uint64_t last_time = time_us_64();
  while (true) {
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;
    dt = std::min(dt, 0.0001f); // clamp dt to avoid spikes after a long pause
    dt = std::max(dt, 1e-6f);   // dt = 0 would make 1/dt infinite and the velocity filter would latch at inf

    if (shared_data.servo_loop_enabled) {
      spin_lock_unsafe_blocking(shared_data.lock);
      double target_mm = shared_data.target_position_mm;
      float target_vel_mm_s = shared_data.target_velocity_mm_s;
      spin_unlock_unsafe(shared_data.lock);

      axis->update_servo(target_mm, dt, 1.0f / dt, target_vel_mm_s);
    }
  }
}

void setup() {
  overclock();

  Logger::instance().begin(921600, false);
  while (!Serial) {}
  delay(100);

  LOG_INFO("Single-axis firmware - starting");
  LOG_INFO("System clock: %i MHz", (int)(clock_get_hz(clk_sys) / 1000 / 1000));

  if (!LittleFS.begin()) {
    LOG_ERROR("Could not mount the filesystem (LittleFS)");
  }

  // hardware SPI for the encoder
  MT6835Encoder::setup_spi(spi0, PIN_ENCODER_SCK, PIN_ENCODER_MOSI, PIN_ENCODER_MISO, 8000000);
  auto* encoder = new MT6835Encoder(spi0, PIN_ENCODER1_CS);

  auto* motor_driver = new TB6612MotorDriver(
    PIN_MOTOR_EN, PIN_M1_PWM_A_POS, PIN_M1_PWM_A_NEG, PIN_MOTOR_PWM_A,
    PIN_MOTOR_EN, PIN_M1_PWM_B_POS, PIN_M1_PWM_B_NEG, PIN_MOTOR_PWM_B
  );

  axis = new AxisController(encoder, motor_driver, MOTOR_POLE_PAIRS, LEAD_SCREW_PITCH_MM);
  axis->init();

  command_processor = new SingleAxisCommandProcessor(axis, &motion_profile, &shared_data, &cascade_corrector);
  command_parser.set_command_processor(command_processor);

  last_profile_update_time_us = time_us_64();
  add_repeating_timer_us(-500, update_motion_profile_isr, nullptr, &motion_profile_timer);

  multicore_launch_core1(main_core1);
  shared_data.servo_loop_enabled = true;

  LOG_INFO("Initialization done. Send G28 to home, M56 to calibrate.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    command_parser.add_input_character(c);
  }
  command_parser.update();

  // --- Drain the telemetry ring buffer as a binary stream ---
  // At most TELEMETRY_DRAIN_BATCH_SIZE points per loop() so incoming commands still get parsed.
  // Stop when the serial TX buffer lacks room for a full packet, rather than block in Serial.write.
  if (shared_data.telemetry_enabled) {
    static constexpr int TELEMETRY_DRAIN_BATCH_SIZE = 50;
    static constexpr int TELEMETRY_PACKET_SIZE = 2 + sizeof(TelemetryPoint);  // header + 28 bytes
    static constexpr uint8_t TELEMETRY_HEADER[2] = {0xAA, 0x56};   // see TelemetryPoint in shared_data.h

    TelemetryPoint tp;
    int drained = 0;
    while (drained < TELEMETRY_DRAIN_BATCH_SIZE &&
           Serial.availableForWrite() >= TELEMETRY_PACKET_SIZE) {
      if (!shared_data.telemetry_buffer.pop(tp, shared_data.lock)) {
        break;  // buffer empty
      }
      Serial.write(TELEMETRY_HEADER, 2);
      Serial.write(reinterpret_cast<const uint8_t*>(&tp), sizeof(TelemetryPoint));
      drained++;
    }
  }
}
