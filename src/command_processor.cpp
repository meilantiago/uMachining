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
  if (c == "M58") {
    // M58 [V<0|1>] [F<0|1>]: V = velocidad con la pendiente de la tabla (1) o con cuentas crudas
    // (0, como antes del 16/10); F = realimentacion anticipada de la velocidad del perfil.
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
    // M59: compensacion de la retencion del motor (anti-cogging). Sin argumentos informa el estado.
    //   S<0|1> activar/desactivar   G<ganancia>   C borrar las tablas
    //   I<indice> V<grados el.>  parte igual en los dos sentidos
    //   J<indice> V<grados el.>  parte que cambia de signo con el sentido (escala con la velocidad, ver R)
    //   R<mm/s> velocidad de referencia de la parte por sentido
    //   P imprimir las tablas       W guardar en flash   L cargar de flash
    ServoController* servo = axis->get_servo_controller();
    const int n = ServoController::COGGING_TABLE_SIZE;
    if (cmd.has_word('C')) {
      for (int i = 0; i < n; i++) servo->cogging_table[i] = servo->cogging_table_dir[i] = 0.0f;
    }
    if (cmd.has_word('I') && cmd.has_word('V')) {
      int i = int(cmd.get_value('I'));
      if (i < 0 || i >= n) { reply = "error: indice fuera de rango\n"; return; }
      servo->cogging_table[i] = cmd.get_value('V') * Constants::DEG2RAD;
    }
    if (cmd.has_word('J') && cmd.has_word('V')) {
      int i = int(cmd.get_value('J'));
      if (i < 0 || i >= n) { reply = "error: indice fuera de rango\n"; return; }
      servo->cogging_table_dir[i] = cmd.get_value('V') * Constants::DEG2RAD;
    }
    if (cmd.has_word('R') && cmd.get_value('R') > 0.0f)
      servo->cogging_dir_ref = cmd.get_value('R') / LEAD_SCREW_PITCH_MM * Constants::TWO_PI_F;
    if (cmd.has_word('G')) servo->cogging_gain = cmd.get_value('G');
    if (cmd.has_word('S')) servo->cogging_enabled = cmd.get_value('S') >= 1.0f;
    if (cmd.has_word('L') && !axis->load_cogging()) { reply = "error: no hay tabla anti-cogging en flash\n"; return; }
    if (cmd.has_word('W') && !axis->store_cogging()) { reply = "error: no se pudo guardar\n"; return; }
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
    snprintf(buf, sizeof(buf), "anti_cogging=%d ganancia=%.3f puntos=%d max=%.3f deg max_sentido=%.3f deg "
             "ref=%.3f mm/s campo_en_cero=%.6f rad\n",
             (int)servo->cogging_enabled, (float)servo->cogging_gain, n, amp * Constants::RAD2DEG,
             amp_dir * Constants::RAD2DEG, servo->cogging_dir_ref / Constants::TWO_PI_F * LEAD_SCREW_PITCH_MM,
             axis->field_angle_at_zero());
    reply += std::string(buf) + "ok\n";
    return;
  }
  if (c == "M121") { axis->dump_trace(); reply = "ok\n"; return; }
  if (c == "M57") {
    // Diagnostico: M57 X<distancia mm> [F<mm/s>] [P<decimacion>]. Barrido en lazo abierto
    // (solo ida) imprimiendo el desfase de conmutacion. Deja el motor energizado con el
    // lazo cerrado apagado: despues mandar M17.
    if (!axis->is_homed() || !axis->is_calibrated()) { reply = "error: eje no homeado/calibrado\n"; return; }
    if (!cmd.has_word('X')) { reply = "error: falta X (distancia en mm)\n"; return; }
    float dist = cmd.get_value('X');
    float end_mm = axis->get_position_mm() + dist;
    if (end_mm < AXIS_MIN_POSITION_MM || end_mm > AXIS_MAX_POSITION_MM) {
      reply = "error: el barrido termina fuera de rango\n";
      return;
    }
    float speed = std::min(cmd.get_value('F', 1.6f), 2.0f);
    int decimation = std::max(1, int(cmd.get_value('P', 4.0f)));

    shared_data->servo_loop_enabled = false;
    sleep_ms(5);
    axis->commutation_sweep(dist, speed, decimation);
    float pos_mm = axis->get_position_mm();
    uint32_t irq_state = save_and_disable_interrupts();
    profile->start(pos_mm, pos_mm, max_velocity_mm_s, max_accel_mm_s2);
    restore_interrupts(irq_state);
    current_target_mm = pos_mm;
    shared_data->target_position_mm = pos_mm;
    shared_data->servo_loop_enabled = true;
    reply = "ok\n";
    return;
  }

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
  // M56 F<0|1>: filtrar la huella del motor en la tabla (por defecto, CALIBRATION_FILTER_MOTOR_HARMONICS)
  set_calibration_motor_filter(cmd.has_word('F') ? cmd.get_value('F') >= 1.0f : CALIBRATION_FILTER_MOTOR_HARMONICS);

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
  shared_data->telemetry_decimation = std::max(1, int(cmd.get_value('P', 1.0f)));
  shared_data->telemetry_enabled = enable;

  if (enable) {
    LOG_INFO("Telemetria binaria activada");
  } else {
    LOG_INFO("Telemetria binaria desactivada");
  }

  reply = "ok\n";
}
