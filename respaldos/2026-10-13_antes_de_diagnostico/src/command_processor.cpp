#include "command_processor.h"
#include "servo_control/cascade_corrector.h"
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
  // siempre aceptamos el comando: si no lo reconocemos, process_command()
  // responde con un error explícito en vez de dejarlo colgado reintentando.
  return true;
}

void SingleAxisCommandProcessor::process_command(const GCodeCommand& cmd, std::string& reply) {
  const std::string& c = cmd.get_command();

  if (c == "G0" || c == "G1") { process_move_command(cmd, reply); return; }
  if (c == "G28") { process_home_command(cmd, reply); return; }
  if (c == "G90") { absolute_mode = true; reply = "ok\n"; return; }
  if (c == "G91") { absolute_mode = false; reply = "ok\n"; return; }

  if (c == "M17") {
    // sin calibracion la conmutacion no sabe donde esta el rotor y el lazo se dispara
    if (!axis->is_calibrated()) { reply = "error: eje no calibrado, correr M56 primero\n"; return; }
    // Sin homing (p.ej. recien encendido con la calibracion de flash) el contador del encoder
    // puede estar corrido un periodo respecto de la tabla: conmutacion desfasada 120/240 grados
    // y el motor empuja para el lado contrario. G28 verifica y corrige eso.
    if (!axis->is_homed()) { reply = "error: eje no homeado, correr G28 (o M56) primero\n"; return; }
    // Rearmar quieto donde esta el carro. Antes el objetivo seguia siendo el destino del
    // ultimo G1 (p.ej. 255 mm tras un corte a mitad de camino): el lazo arrancaba con
    // cientos de mm de error y el corte por falla (>1 mm durante 0.5 s) volvia a saltar
    // a los 0.5 s. Ahora hay que volver a mandar el G1 despues del M17.
    float pos_mm = axis->get_position_mm();
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(pos_mm, pos_mm, max_velocity_mm_s, max_accel_mm_s2);
    restore_interrupts(irq_state);
    current_target_mm = pos_mm;
    shared_data->target_position_mm = pos_mm;
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
  if (c == "M121") { axis->dump_trace(); reply = "ok\n"; return; }

  reply = "error: unknown command '" + c + "'\n";
}

void SingleAxisCommandProcessor::process_move_command(const GCodeCommand& cmd, std::string& reply) {
  if (!axis->is_homed() || !axis->is_calibrated()) {
    reply = "error: eje no homeado/calibrado, correr G28 y M56 primero\n";
    return;
  }

  if (!cmd.has_word('X')) {
    reply = "error: falta parametro X\n";
    return;
  }

  float x = cmd.get_value('X');
  float target_mm = absolute_mode ? x : (current_target_mm + x);

  if (target_mm < AXIS_MIN_POSITION_MM || target_mm > AXIS_MAX_POSITION_MM) {
    reply = "error: posicion fuera de rango [" +
            std::to_string(AXIS_MIN_POSITION_MM) + ", " +
            std::to_string(AXIS_MAX_POSITION_MM) + "] mm\n";
    return;
  }

  float feedrate = cmd.get_value('F', max_velocity_mm_s);
  feedrate = std::min(feedrate, max_velocity_mm_s);

  // El timer de Core 0 llama a profile->update() de forma asíncrona (ISR). Evitamos
  // que profile->start() sea interrumpido a mitad de escritura deshabilitando IRQs
  // brevemente (mismo tipo de precaución que el spinlock usado para SharedData).
  // El movimiento nuevo arranca desde donde va el objetivo ahora. Antes arrancaba desde el
  // destino del G1 anterior: un G1 mandado a mitad de otro hacia saltar el objetivo de golpe.
  uint32_t irq_state = save_and_disable_interrupts();
  profile->start(profile->get_current_position(), target_mm, feedrate, max_accel_mm_s2);
  restore_interrupts(irq_state);

  current_target_mm = target_mm;
  reply = "ok\n";
}

// El homing contra el tope positivo (D1) no esta implementado: la calibracion barre siempre en
// sentido positivo desde el home y los mm positivos apuntan siempre para ese lado. Con D1 el
// eje creeria estar en 0 parado contra el tope del otro extremo.
static bool homing_direction_supported(const GCodeCommand& cmd, std::string& reply) {
  if (cmd.get_value('D', -1.0f) >= 0.0f) {
    reply = "error: solo se puede homear contra el tope negativo (D-1); D1 no esta implementado\n";
    return false;
  }
  return true;
}

void SingleAxisCommandProcessor::process_home_command(const GCodeCommand& cmd, std::string& reply) {
  if (!homing_direction_supported(cmd, reply)) return;
  float direction_sign = -1.0f;
  bool search_stop = !cmd.has_word('H');  // G28 H: home aca mismo, sin tope (motor desacoplado)

  // Core 1 no debe tocar el encoder/driver mientras homing corre bloqueante en Core 0.
  shared_data->servo_loop_enabled = false;
  sleep_ms(5); // margen para que Core 1 termine cualquier update() en curso

  bool ok = axis->home(direction_sign, search_stop);

  if (ok) {
    current_target_mm = 0.0f;
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(0.0f, 0.0f, max_velocity_mm_s, max_accel_mm_s2); // deja el perfil quieto en 0
    restore_interrupts(irq_state);
    shared_data->target_position_mm = 0.0f;
    cascade_corrector->reset();  // limpiar integral para evitar windup residual
  }

  shared_data->servo_loop_enabled = true;
  reply = ok ? "ok\n" : "error: homing fallido\n";
}

void SingleAxisCommandProcessor::process_tune_pid_command(const GCodeCommand& cmd, std::string& reply) {
  bool has_all = cmd.has_word('A') && cmd.has_word('B') && cmd.has_word('C') &&
                 cmd.has_word('D') && cmd.has_word('F');
  if (!has_all) {
    reply = "error: parametros esperados A(pos_kp) B(pos_ki) C(vel_kp) D(vel_ki) F(vel_lowpass_tc)\n";
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
  bool search_stop = !cmd.has_word('H');  // M56 H: calibrar aca mismo, sin buscar tope

  shared_data->servo_loop_enabled = false;
  sleep_ms(5);

  bool ok = axis->calibrate(direction_sign, store_to_flash, print_measurements, search_stop);

  if (ok) {
    current_target_mm = 0.0f;
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(0.0f, 0.0f, max_velocity_mm_s, max_accel_mm_s2);
    restore_interrupts(irq_state);
    shared_data->target_position_mm = 0.0f;
    cascade_corrector->reset();  // limpiar integral para evitar windup residual
  }

  shared_data->servo_loop_enabled = true;
  reply = ok ? "ok\n" : "error: calibracion fallida\n";
}

void SingleAxisCommandProcessor::process_report_position_command(std::string& reply) {
  // posicion calculada por Core 1: leer el encoder desde aca chocaria con sus lecturas SPI
  float pos_mm = axis->get_position_mm();
  char buf[64];
  snprintf(buf, sizeof(buf), "X:%.4f\n", pos_mm);
  reply = std::string(buf) + "ok\n";
}

void SingleAxisCommandProcessor::process_report_status_command(std::string& reply) {
  MT6835Encoder* encoder = axis->get_encoder();
  float encoder_angle_deg = encoder->get_last_abs_angle() * Constants::RAD2DEG;
  int32_t crc_errors = encoder->is_crc_enabled() ? (int32_t)encoder->get_crc_error_count(false) : -1;

  char buf[256];
  snprintf(buf, sizeof(buf),
    "homed=%d calibrated=%d target_mm=%.4f pos_mm=%.4f err_mm=%.4f enc_deg=%.3f crc_errors=%d\n",
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
      "ultimo corte (%lu en total): target_mm=%.4f pos_mm=%.4f err_mm=%.4f vel=%.3f rad/s out=%.3f status=%u\n",
      (unsigned long)f.count, f.target_mm, f.pos_mm, f.target_mm - f.pos_mm, f.vel, f.out, (unsigned)f.status);
    reply += std::string(buf);
  }

  reply += "ok\n";
}

void SingleAxisCommandProcessor::process_telemetry_command(const GCodeCommand& cmd, std::string& reply) {
  if (!cmd.has_word('S')) {
    reply = "error: M120 requiere parametro S (1=activar, 0=desactivar)\n";
    return;
  }

  bool enable = cmd.get_value('S') >= 1.0f;
  shared_data->telemetry_enabled = enable;

  if (enable) {
    LOG_INFO("Telemetria binaria activada");
  } else {
    LOG_INFO("Telemetria binaria desactivada");
  }

  reply = "ok\n";
}
