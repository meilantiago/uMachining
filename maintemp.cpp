// --------------------------------------------------------------------------------------
// Firmware de eje único (X) para micromaquinado, adaptado de MicroManipulatorStepper
// (0x23), firmware/MotionControllerRP. Ver README.md para el detalle de qué módulos
// se reusaron tal cual y cuáles se reescribieron para 1 solo eje.
//
// Arquitectura (igual idea que el original, pero con 1 sola junta y sin cinemática):
//   Core 0: lee comandos G-code por serie, corre un timer a 2 kHz que avanza el
//           perfil trapezoidal y escribe la posición objetivo en SharedData.
//   Core 1: en loop libre, lee SharedData y corre el lazo servo (encoder -> PID
//           en cascada -> comutación FOC) tan rápido como puede.
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

// El sistema real tiene un ÚNICO encoder MT6835 frente a una rueda multipolar magnética.
// La lectura lineal (el avance real del husillo) se infiere aquí directamente del
// controlador del eje.
float read_linear_encoder() {
  return axis->get_position_mm();
}

//*** FUNCTIONS *************************************************************************

void overclock() {
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  busy_wait_us(10 * 1000);
  set_sys_clock_khz(250000, true);
}

// Corre en un timer de hardware cada 500us (2 kHz). Avanza el perfil trapezoidal
// y publica la nueva posición objetivo para que Core 1 la use.
bool update_motion_profile_isr(repeating_timer_t* timer) {
  uint64_t time_us = time_us_64();
  float dt = float(time_us - last_profile_update_time_us) * 1e-6f;
  last_profile_update_time_us = time_us;

  float pos_mm, vel_mm_s;
  motion_profile.update(dt, pos_mm, vel_mm_s);

  // --- Lazo externo de cascada (encoder lineal) ---
  // Lee la posición del encoder lineal y aplica la corrección PI.
  // Con el mock, linear_pos ≈ motor_pos y la corrección es mínima.
  float linear_pos = read_linear_encoder();
  if (shared_data.cogging_learner.active)
    shared_data.cogging_learner.add(pos_mm, linear_pos, vel_mm_s);
  float corrected_pos = pos_mm;  // default: sin corrección
  if (shared_data.cascade_corrector_enabled) {
    corrected_pos = cascade_corrector.update(pos_mm, linear_pos, dt);
  }

  if (spin_try_lock_unsafe(shared_data.lock)) {
    shared_data.target_position_mm = corrected_pos;
    shared_data.target_velocity_mm_s = vel_mm_s;
    spin_unlock_unsafe(shared_data.lock);
  }

  // --- Telemetría: empaquetar estado actual si está habilitada ---
  // Usa try_push (non-blocking): si el lock está tomado o el buffer lleno,
  // se descarta este punto silenciosamente. No se puede bloquear en ISR.
  // Todo lo que se lee aca son valores cacheados que escribe Core 1 (sin SPI).
  static uint32_t telemetry_tick = 0;
  static uint16_t telemetry_seq = 0;
  if (shared_data.telemetry_enabled && ++telemetry_tick >= shared_data.telemetry_decimation) {
    telemetry_tick = 0;
    ServoController* servo = axis->get_servo_controller();
    TelemetryPoint tp;
    tp.timestamp_us = static_cast<uint32_t>(time_us);   // truncar a 32 bits
    tp.target_mm    = pos_mm;                            // referencia pre-corrección (perfil puro)
    tp.motor_pos_mm = linear_pos;                        // = axis->get_position_mm()
    tp.vel          = servo->get_velocity();
    tp.out          = servo->get_output();
    tp.chip_raw     = axis->get_encoder()->get_last_chip_raw_angle();
    tp.status       = axis->get_encoder()->get_status();
    tp.flags        = (servo->is_motor_update_enabled() ? 1 : 0) | (axis->has_servo_fault() ? 2 : 0);
    tp.seq          = telemetry_seq++;
    shared_data.telemetry_buffer.try_push(tp, shared_data.lock);
  }

  return true; // seguir repitiendo
}

// Core 1: lazo servo dedicado (encoder + PID en cascada + comutación FOC).
void main_core1() {
  LOG_INFO("Core 1: arrancando lazo servo...");
  // Permite que Core 0 pause este nucleo mientras escribe la flash (M56 S, ver store_calibration)
  multicore_lockout_victim_init();

  uint64_t last_time = time_us_64();
  while (true) {
    uint64_t time_us = time_us_64();
    float dt = float(time_us - last_time) * 1e-6f;
    last_time = time_us;
    dt = std::min(dt, 0.0001f); // limitar dt para evitar picos tras una pausa larga
    dt = std::max(dt, 1e-6f);   // dt = 0 daria 1/dt infinito (velocidad inf que queda pegada en el filtro)

    if (shared_data.servo_loop_enabled) {
      spin_lock_unsafe_blocking(shared_data.lock);
      float target_mm = shared_data.target_position_mm;
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

  LOG_INFO("Firmware de eje unico - iniciando");
  LOG_INFO("System clock: %i MHz", (int)(clock_get_hz(clk_sys) / 1000 / 1000));

  if (!LittleFS.begin()) {
    LOG_ERROR("No se pudo montar el filesystem (LittleFS)");
  }

  // hardware SPI compartido por el encoder
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

  LOG_INFO("Inicializacion terminada. Enviar G28 para homear, M56 para calibrar.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    command_parser.add_input_character(c);
  }
  command_parser.update();

  // --- Drenar ring buffer de telemetría como stream binario ---
  // Enviar hasta TELEMETRY_DRAIN_BATCH_SIZE puntos por iteración de loop() para
  // no acaparar CPU y dejar que command_parser procese comandos entrantes.
  // Backpressure: si el buffer serial de salida está lleno, parar hasta la
  // próxima iteración (Serial.write no bloquea si hay espacio, pero no queremos
  // esperar si no lo hay).
  if (shared_data.telemetry_enabled) {
    static constexpr int TELEMETRY_DRAIN_BATCH_SIZE = 50;
    static constexpr int TELEMETRY_PACKET_SIZE = 2 + sizeof(TelemetryPoint);  // header + 28 bytes
    static constexpr uint8_t TELEMETRY_HEADER[2] = {0xAA, 0x55};

    TelemetryPoint tp;
    int drained = 0;
    while (drained < TELEMETRY_DRAIN_BATCH_SIZE &&
           Serial.availableForWrite() >= TELEMETRY_PACKET_SIZE) {
      if (!shared_data.telemetry_buffer.pop(tp, shared_data.lock)) {
        break;  // buffer vacío
      }
      Serial.write(TELEMETRY_HEADER, 2);
      Serial.write(reinterpret_cast<const uint8_t*>(&tp), sizeof(TelemetryPoint));
      drained++;
    }
  }
}
