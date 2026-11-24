#include "homing_controller.h"
#include "utilities/math_constants.h"
#include "utilities/logging.h"
#include "pico/time.h"
#include <algorithm>

static constexpr int EVAL_WINDOWS_PER_CHECK = 17;   // 17 x 0.72 grados = ~1 periodo del encoder
// contra el tope el rotor vibra y salta de polo, y el encoder registra algo de
// movimiento: se midio ~0.33 en un tope real. En movimiento libre da ~1.
static constexpr float STALL_RATIO = 0.5f;
// El tope se confirma con varias ventanas lentas SEGUIDAS. Un patinazo suelto del rotor
// (pierde un ciclo electrico = 7.2 grados de eje dentro de una ventana de ~12) tambien da
// una relacion < 0.5, pero en la ventana siguiente el eje vuelve a avanzar normal. Contra
// el tope real, en cambio, todas las ventanas siguientes dan lento.
static constexpr int STALL_CONFIRM_CHECKS = 3;
// rampa de velocidad al arrancar: arrancar de golpe en lazo abierto puede hacer perder pasos
static constexpr float START_RAMP_S = 0.3f;
static constexpr float MIN_HEALTHY_RATIO = 0.7f;
static constexpr float MAX_HEALTHY_RATIO = 1.3f;
static constexpr int MAX_INCOHERENT_CHECKS = 3;

HomingController::HomingController() {
  retract_field_velocity = 100.0f; // rad per second
  retract_field_angle = Constants::TWO_PI_F*0.25f;
}

bool HomingController::run_blocking(ServoController* servo_controller, 
                                    float motor_velocity, 
                                    float search_range_angle, 
                                    float current, 
                                    float encoder_angle_to_motor_angle,
                                    float retract_angle_rad)
{
  start(servo_controller, motor_velocity, search_range_angle, current, 
        encoder_angle_to_motor_angle, retract_angle_rad);

  while(is_finished() == false) {
    update();
  }

  finalize();
  return is_successful();
}

void HomingController::start(ServoController* servo_controller, 
                            float velocity, 
                            float search_range_angle, 
                            float current, 
                            float encoder_angle_to_motor_angle,
                            float retract_angle_rad)
{
  float pole_pair_count = servo_controller->get_pole_pair_count();
  float field_angle_to_encoder_angle = 1.0f / pole_pair_count / encoder_angle_to_motor_angle;

  if(retract_angle_rad > 0.0f)
    HomingController::retract_field_angle = retract_angle_rad * pole_pair_count;

  // field_angle_to_rotor_angle = 1.0 / pole_pair_count
  // encoder_angle_to_rotor_angle = encoder_period_pitch/encoder_radius
  // field_angle_to_encoder_angle = field_angle_to_rotor_angle/encoder_angle_to_rotor_angle
  
  eval_field_angle_delta = Constants::TWO_PI_F*0.1f;
  expected_encoder_delta = eval_field_angle_delta * field_angle_to_encoder_angle;
  
  servo_ctrl = servo_controller;
  field_velocity = velocity * pole_pair_count;
  field_angle_search_range = search_range_angle * pole_pair_count;
  homing_current = current;

  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  // perform 'soft start'
  servo_ctrl->set_motor_enabled(true, false);

  initial_current = servo_ctrl->get_motor_driver().get_amplitude();
  servo_ctrl->get_motor_driver().set_amplitude_smooth(homing_current, 100);
  search_failed = false;

  // motor_driver.rotate_field(Constants::TWO_PI_F*0.5f * (field_velocity>0.0f ? -1.0f : 1.0f), 12.0f);
  start_field_angle = fmodf(motor_driver.get_field_angle(), Constants::TWO_PI_F);

  last_eval_encoder_angle = encoder.read_abs_angle();
  last_time = 0;
  start_time = 0;
  stall_checks = 0;

  state = State::Homing;
}

void HomingController::update() {
  if (state != State::Homing)
    return;

  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  uint64_t time_us = time_us_64();
  if(last_time == 0) last_time = start_time = time_us;
  float dt = float(time_us - last_time) * 1e-6f;
  last_time = time_us;
  float ramp = std::min(1.0f, float(time_us - start_time) * 1e-6f / START_RAMP_S);

  // Move motor and read encoder
  field_angle_offset += field_velocity * ramp * dt;
  motor_driver.set_field_angle(start_field_angle+field_angle_offset);

  float encoder_angle = encoder.read_abs_angle();

  // Se compara lo que avanzo el encoder contra lo que avanzo el campo sobre un
  // periodo completo del encoder (~12 grados de eje): en tramos mas cortos la
  // no-linealidad de la rueda hace variar la relacion entre 0 y 2 aunque todo este
  // bien, y la zona de campo debil puede hacer parecer quieto al encoder.
  //  - relacion < STALL_RATIO:   el eje no avanza -> tope encontrado
  //  - relacion ~1:               movimiento normal
  //  - otra cosa varias veces:    el encoder no sigue al motor (rueda de imanes
  //                               patinando sobre el eje, etc.) -> se aborta en vez
  //                               de seguir empujando contra el tope
  if (fabs(last_eval_field_angle_offset - field_angle_offset) > eval_field_angle_delta * EVAL_WINDOWS_PER_CHECK) {
    float encoder_delta = encoder_angle - last_eval_encoder_angle;
    float ratio = fabsf(encoder_delta / (expected_encoder_delta * EVAL_WINDOWS_PER_CHECK));
    last_eval_encoder_angle = encoder_angle;
    last_eval_field_angle_offset = field_angle_offset;

    if (ratio < STALL_RATIO) {
      if (++stall_checks >= STALL_CONFIRM_CHECKS) {
        LOG_INFO("Homing: tope detectado tras %.2f vueltas de motor (relacion %.2f en %i ventanas seguidas)",
                 fabs(field_angle_offset) / servo_ctrl->get_pole_pair_count() / Constants::TWO_PI_F,
                 ratio, stall_checks);
        on_endstop_detected();
      }
      return;
    }
    if (stall_checks > 0)
      LOG_INFO("Homing: %i ventana(s) lenta(s) sin tope (patinazo del rotor?), sigue buscando", stall_checks);
    stall_checks = 0;

    if (ratio > MIN_HEALTHY_RATIO && ratio < MAX_HEALTHY_RATIO) {
      incoherent_checks = 0;
    } else if (++incoherent_checks >= MAX_INCOHERENT_CHECKS) {
      LOG_ERROR("Homing abortado: encoder no sigue al motor (relacion %f, deberia ~1). Rueda floja?", ratio);
      search_failed = true;
      on_endstop_detected();
      return;
    }
  }

  if (fabs(field_angle_offset) > field_angle_search_range) {
    LOG_INFO("End stop not detected");
    search_failed = true;
    on_endstop_detected(); // marca state=Done (search_failed ya desactiva el motor ahi mismo)
  }
}

void HomingController::on_endstop_detected() {
  state = State::Done;
  auto& motor_driver = servo_ctrl->get_motor_driver();
  auto& encoder = servo_ctrl->get_encoder();

  if(search_failed) {
    servo_ctrl->set_motor_enabled(false, false);
    return;
  }

  // reset encoder period, the remainder will provide a very repeatable position reference
  encoder.read_abs_angle();
  servo_ctrl->get_encoder().reset_abs_angle_period();

  // the motor is currently held against the end stop by the field, defining a geometric reference
  home_encoder_angle = encoder.read_abs_angle();
  LOG_DEBUG("home_encoder_angle=%f deg", home_encoder_angle*Constants::RAD2DEG);
  if(home_encoder_angle < Constants::TWO_PI_F*0.01 || home_encoder_angle > Constants::TWO_PI_F*0.99)
    LOG_WARNING("encoder angle at home position close to wrap around point !");
}

void HomingController::finalize() {
  if (search_failed)
    return; // el motor ya quedo desactivado en on_endstop_detected(), no hay tope del cual retroceder

   auto& motor_driver = servo_ctrl->get_motor_driver();
   float pole_pair_count = servo_ctrl->get_pole_pair_count();

  // restore previous motor current before retracting, otherwise the retract move
  // (the fastest move in the whole homing/calibration sequence) runs at the low
  // homing current and can make the rotor slip/stall.
  motor_driver.set_amplitude_smooth(initial_current, 100);

  // back off from home position. Hasta confirmar el bloqueo el campo puede haber
  // avanzado STALL_CONFIRM_CHECKS+1 chequeos completos contra el tope: se deshace eso primero.
  float retract = retract_field_angle + (STALL_CONFIRM_CHECKS + 1) * EVAL_WINDOWS_PER_CHECK * eval_field_angle_delta;
  motor_driver.rotate_field(retract * (field_velocity>0.0f ? -1.0f : 1.0f),
                            retract_field_velocity, [this](){
                              // update encoder so it doesnt miss a period
                              servo_ctrl->get_encoder().read_abs_angle();
                            });
}

bool HomingController::is_finished() const {
  return state == State::Done;
}

bool HomingController::is_successful() const {
  return state == State::Done && !search_failed;
}

float HomingController::get_home_encoder_angle() const {
  return home_encoder_angle;
}


