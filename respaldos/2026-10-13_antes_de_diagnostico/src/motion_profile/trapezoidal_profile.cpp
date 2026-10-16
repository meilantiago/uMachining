#include "trapezoidal_profile.h"
#include <cmath>
#include <algorithm>

void TrapezoidalProfile::start(float new_start_pos, float new_target_pos, float requested_max_vel, float requested_max_accel) {
  start_pos = new_start_pos;
  target_pos = new_target_pos;
  current_pos = new_start_pos;
  distance = std::fabs(target_pos - start_pos);
  direction = (target_pos >= start_pos) ? 1.0f : -1.0f;

  max_accel = std::max(requested_max_accel, 1e-6f);
  elapsed = 0.0f;

  if (distance < 1e-6f) {
    // ya estamos en el destino, no hay nada que mover
    max_vel = 0.0f;
    t_accel = t_cruise = t_total = 0.0f;
    active = false;
    return;
  }

  // distancia recorrida acelerando desde 0 hasta requested_max_vel
  float accel_distance = 0.5f * requested_max_vel * requested_max_vel / max_accel;

  if (2.0f * accel_distance <= distance) {
    // perfil trapezoidal: alcanza velocidad de crucero
    max_vel = requested_max_vel;
    t_accel = max_vel / max_accel;
    float cruise_distance = distance - 2.0f * accel_distance;
    t_cruise = cruise_distance / max_vel;
  } else {
    // recorrido corto: perfil triangular, no llega a la velocidad pedida
    max_vel = std::sqrt(max_accel * distance);
    t_accel = max_vel / max_accel;
    t_cruise = 0.0f;
  }

  t_total = 2.0f * t_accel + t_cruise;
  active = true;
}

bool TrapezoidalProfile::update(float dt, float& out_pos, float& out_vel) {
  if (!active) {
    out_pos = current_pos = target_pos;
    out_vel = 0.0f;
    return false;
  }

  elapsed += dt;

  float pos_along_path = 0.0f; // distancia recorrida desde start_pos, en [0, distance]
  float vel_along_path = 0.0f;

  if (elapsed >= t_total) {
    // movimiento terminado
    pos_along_path = distance;
    vel_along_path = 0.0f;
    active = false;
  } else if (elapsed < t_accel) {
    // rampa de aceleración
    pos_along_path = 0.5f * max_accel * elapsed * elapsed;
    vel_along_path = max_accel * elapsed;
  } else if (elapsed < t_accel + t_cruise) {
    // tramo a velocidad constante
    float accel_distance = 0.5f * max_accel * t_accel * t_accel;
    float t_in_cruise = elapsed - t_accel;
    pos_along_path = accel_distance + max_vel * t_in_cruise;
    vel_along_path = max_vel;
  } else {
    // rampa de desaceleración
    float t_in_decel = elapsed - t_accel - t_cruise;
    float accel_distance = 0.5f * max_accel * t_accel * t_accel;
    float cruise_distance = max_vel * t_cruise;
    pos_along_path = accel_distance + cruise_distance +
                     (max_vel * t_in_decel - 0.5f * max_accel * t_in_decel * t_in_decel);
    vel_along_path = max_vel - max_accel * t_in_decel;
  }

  out_pos = current_pos = start_pos + direction * pos_along_path;
  out_vel = direction * vel_along_path;
  return active;
}

float TrapezoidalProfile::get_current_position() const {
  return current_pos;
}

bool TrapezoidalProfile::is_idle() const {
  return !active;
}

float TrapezoidalProfile::get_target_position() const {
  return target_pos;
}
