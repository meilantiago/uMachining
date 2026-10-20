#include <LittleFS.h>
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
  servo_controller = new ServoController(*motor_driver, *encoder, (int32_t)motor_pole_pairs);
}

AxisController::~AxisController() {
  delete servo_controller;
  delete motor_driver;
  delete encoder;
}

void AxisController::init() {
  if (!encoder->init(0x5, 0x4)) {
    LOG_ERROR("AxisController: fallo al inicializar el encoder");
  }
  encoder->set_crc_enabled(ENABLE_ENCODER_CRC);

  servo_controller->init(MOTOR_MAX_CURRENT_FACTOR);
  servo_controller->set_motor_enabled(false, false);

  load_calibration();
  if (load_cogging()) {
    servo_controller->cogging_enabled = COGGING_ENABLED_AT_BOOT;
    LOG_INFO("AxisController: tabla anti-cogging cargada desde flash (%s)",
             COGGING_ENABLED_AT_BOOT ? "activa" : "inactiva, M59 S1 para activarla");
  }
}

bool AxisController::home(float direction_sign, bool search_stop) {
  // el homing maneja el campo directamente: el lazo cerrado queda apagado (M17 lo rearma)
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
      LOG_ERROR("AxisController: homing fallido (no se detecto el tope mecanico)");
      servo_controller->resync();
      return false;
    }
  } else {
    LOG_WARNING("AxisController: home en la posicion actual, SIN buscar tope. Solo para pruebas con el "
                "motor desacoplado: con el carro puesto, 0..255 mm no corresponde al recorrido real.");
    servo_controller->set_motor_enabled(true, false);  // energiza con el campo actual: el rotor se alinea
    sleep_ms(200);
    encoder->read_abs_angle_raw();
    encoder->reset_abs_angle_period();
  }

  // El homing fija el contador del encoder modulo un periodo, y eso alcanza para la posicion
  // pero no para la conmutacion (ver COMMUTATION_CHECK_CYCLES en hw_config.h).
  if (calibrated && !verify_commutation(-direction_sign, true)) {
    LOG_ERROR("AxisController: la conmutacion no coincide con la calibracion guardada. Correr M56.");
    servo_controller->set_motor_enabled(false, false);
    servo_controller->resync();
    return false;
  }

  servo_controller->resync();
  last_position_mm = motor_pos_rad_to_mm(servo_controller->get_position());
  homed = true;
  LOG_INFO("AxisController: homing OK (pos = %.4f mm). Lazo cerrado apagado: M17 para habilitarlo.",
           last_position_mm);
  return true;
}

bool AxisController::verify_commutation(float away_from_stop_sign, bool allow_period_correction) {
  float offset[3], lag, consistency;
  measure_commutation_offsets(*servo_controller, away_from_stop_sign, COMMUTATION_CHECK_CYCLES,
                              CALIBRATION_FIELD_VELOCITY, offset, lag, consistency);

  // offset[1] = contador como esta; offset[0] / offset[2] = corrido -1 / +1 periodo
  int best = 1;
  if (allow_period_correction) {
    for (int i = 0; i < 3; i++)
      if (fabsf(offset[i]) < fabsf(offset[best])) best = i;
  }

  LOG_INFO("Conmutacion: desfase %.1f deg electricos (con el encoder corrido -1 periodo: %.1f, +1: %.1f), "
           "atraso del rotor %.1f deg, consistencia %.2f",
           offset[1] * Constants::RAD2DEG, offset[0] * Constants::RAD2DEG, offset[2] * Constants::RAD2DEG,
           lag * Constants::RAD2DEG, consistency);

  if (consistency < 0.7f) {
    LOG_ERROR("Conmutacion: el desfase cambia durante el barrido (consistencia %.2f): el encoder no sigue "
              "al rotor (rueda floja, campo debil, rotor trabado?)", consistency);
    return false;
  }
  if (fabsf(offset[best]) > COMMUTATION_MAX_OFFSET_DEG * Constants::DEG2RAD) {
    LOG_ERROR("Conmutacion: desfase de %.1f deg electricos, el maximo aceptado es %.1f",
              offset[best] * Constants::RAD2DEG, COMMUTATION_MAX_OFFSET_DEG);
    return false;
  }

  int k = best - 1;
  if (k != 0) {
    encoder->read_abs_angle_raw();
    encoder->reset_abs_angle(encoder->get_last_abs_raw_angle() + MT6835Encoder::AbsRawAngleType(k) * MT6835_CPR);
    LOG_WARNING("Conmutacion: el contador del encoder habia quedado corrido %d periodo(s) (desfase de %.1f deg "
                "electricos). Corregido: ahora %.1f deg.", k, offset[1] * Constants::RAD2DEG,
                offset[best] * Constants::RAD2DEG);
  }
  return true;
}

bool AxisController::calibrate(float homing_direction_sign, bool store_to_flash, bool print_measurements,
                               bool search_stop) {
  // la tabla nueva se arma sobre la referencia de este homing: no se verifica contra la anterior
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
    LOG_ERROR("AxisController: calibracion fallida");
    calibrated = false;
    return false;
  }

  servo_controller->set_enc_to_pos_lut(encoder_raw_to_motor_pos_lut);
  servo_controller->set_pos_to_field_lut(motor_pos_to_field_angle_lut);

  // Autoverificacion: con la tabla recien medida el desfase tiene que dar ~0 sin corregir nada.
  if (!verify_commutation(-homing_direction_sign, false)) {
    LOG_ERROR("AxisController: la tabla recien medida no conmuta bien. Calibracion descartada.");
    servo_controller->resync();
    return false;
  }
  servo_controller->resync();
  last_position_mm = motor_pos_rad_to_mm(servo_controller->get_position());
  calibrated = true;

  if (store_to_flash) {
    store_calibration();
  }

  LOG_INFO("AxisController: calibracion OK");
  return true;
}

void AxisController::update_servo(float target_position_mm, float dt, float one_over_dt,
                                  float target_velocity_mm_s) {
  float target_motor_pos_rad = mm_to_motor_pos_rad(target_position_mm);
  float velocity_ff = target_velocity_mm_s / lead_screw_pitch_mm * Constants::TWO_PI_F;
  servo_controller->update(target_motor_pos_rad, dt, one_over_dt, velocity_ff);
  last_position_mm = motor_pos_rad_to_mm(servo_controller->get_position());

  // Corte de seguridad: si el eje queda lejos del objetivo mucho tiempo, el encoder
  // probablemente no sigue al rotor (rueda de imanes patinando, zona de campo debil)
  // y la conmutacion ya no esta alineada: el motor solo empujaria sin avanzar.
  constexpr float FAULT_ERROR_MM = 1.0f;
  constexpr uint64_t FAULT_TIME_US = 500000;
  if (!servo_controller->is_motor_update_enabled()) {
    fault_since_us = 0;
    return;
  }
  uint64_t now_us = time_us_64();

  // La traza se congela solo en el corte por falla, asi guarda los ~2 s previos al corte.
  // (Antes se congelaba 300 ms despues de que el error pasara de 0.1 mm por primera vez:
  // en un movimiento largo eso pasa casi al arrancar y la traza no mostraba el corte.)
  loops_since_trace++;
  if (!trace_frozen && now_us - last_trace_us >= 1000) {
    TracePoint& p = trace[trace_head];
    p.t_us = uint32_t(now_us);
    p.target_mm = target_position_mm;
    p.pos_mm = last_position_mm;
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

  if (fabsf(target_position_mm - last_position_mm) < FAULT_ERROR_MM) {
    fault_since_us = 0;
  } else if (fault_since_us == 0) {
    fault_since_us = now_us;
  } else if (now_us - fault_since_us > FAULT_TIME_US) {
    last_fault.count++;
    last_fault.target_mm = target_position_mm;
    last_fault.pos_mm = last_position_mm;
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
  trace_frozen = true;  // que Core 1 no escriba mientras se imprime
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
  // energiza con el campo donde la tabla dice que esta el rotor (igual que M17)
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
  last_position_mm = motor_pos_rad_to_mm(servo_controller->get_position());
}

void AxisController::set_motor_enabled(bool enable) {
  // Primero sacarle el campo a Core 1 y esperar que termine el update() en curso (~45 us).
  // Antes la rampa de corriente (Core 0) y la conmutacion (Core 1) escribian el PWM a la vez.
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

float AxisController::get_position_mm() const {
  return last_position_mm;
}

float AxisController::get_position_error_mm() const {
  // conversion lineal (proporcional), no hace falta restar un offset en 0
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

// --- conversiones mm <-> rad segun el paso del husillo -------------------------------
// una vuelta completa del motor (2*PI rad) avanza el carro 'lead_screw_pitch_mm' mm.

float AxisController::mm_to_motor_pos_rad(float pos_mm) const {
  return (pos_mm / lead_screw_pitch_mm) * Constants::TWO_PI_F;
}

float AxisController::motor_pos_rad_to_mm(float motor_pos_rad) const {
  return (motor_pos_rad / Constants::TWO_PI_F) * lead_screw_pitch_mm;
}

// --- persistencia de calibracion en flash (LittleFS) ----------------------------------

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
    LOG_WARNING("AxisController: no se encontraron datos de calibracion guardados. Correr M56.");
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

  LOG_INFO("AxisController: calibracion cargada desde flash (size=%i,%i)",
           enc_to_pos_lut.size(), pos_to_field_lut.size());
  return true;
}

bool AxisController::store_calibration() {
  // Mientras se borra/escribe la flash, Core 1 no puede ejecutar codigo desde ella. LittleFS
  // intenta pausarlo con rp2040.idleOtherCore(), pero eso solo funciona si Core 1 se arranco
  // con setup1()/loop1() de Arduino; aca se arranca con multicore_launch_core1() y la pausa
  // no hacia nada: M56 S colgaba la placa al grabar (16/10/2026). Se pausa con el mecanismo
  // del SDK (Core 1 llama a multicore_lockout_victim_init() al arrancar, ver main.cpp).
  if (!multicore_lockout_start_timeout_us(200000)) {
    LOG_ERROR("AxisController: no se pudo pausar Core 1, la calibracion NO se guardo en flash");
    return false;
  }
  bool ok = true;
  ok &= save_lut_to_file(servo_controller->get_enc_to_pos_lut(), enc_to_pos_lut_filename().c_str());
  ok &= save_lut_to_file(servo_controller->get_pos_to_field_lut(), pos_to_field_lut_filename().c_str());
  multicore_lockout_end_blocking();
  if (ok) LOG_INFO("AxisController: calibracion guardada en flash");
  else LOG_ERROR("AxisController: error al guardar la calibracion en flash");
  return ok;
}

// --- tabla de compensacion de la retencion del motor (M59) ---------------------------

static const char* COGGING_FILE = "axis_cogging.dat";
static constexpr uint32_t COGGING_MAGIC = 0x434F4732;   // "COG2": tabla, tabla por sentido, velocidad de referencia

float AxisController::field_angle_at_zero() {
  return servo_controller->motor_pos_to_field_angle(0.0f);
}

bool AxisController::store_cogging() {
  // misma precaucion que store_calibration(): Core 1 no puede correr desde la flash mientras se escribe
  if (!multicore_lockout_start_timeout_us(200000)) {
    LOG_ERROR("AxisController: no se pudo pausar Core 1, la tabla anti-cogging NO se guardo");
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
  if (ok) LOG_INFO("AxisController: tabla anti-cogging guardada en flash");
  else LOG_ERROR("AxisController: error al guardar la tabla anti-cogging");
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
    LOG_WARNING("AxisController: el archivo de la tabla anti-cogging no es valido");
    return false;
  }
  for (uint32_t i = 0; i < n; i++) {
    servo_controller->cogging_table[i] = tmp[i];
    servo_controller->cogging_table_dir[i] = tmp_dir[i];
  }
  servo_controller->cogging_dir_ref = ref;
  return true;
}
