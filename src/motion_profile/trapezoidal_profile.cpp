#include "trapezoidal_profile.h"
#include <cmath>
#include <algorithm>

void TrapezoidalProfile::start(double new_start_pos, double new_target_pos, float requested_max_vel, float requested_max_accel) {
  start_pos = new_start_pos;
  target_pos = new_target_pos;
  current_pos = new_start_pos;
  distance = std::fabs(target_pos - start_pos);   // double
  direction = (target_pos >= start_pos) ? 1.0f : -1.0f;

  max_accel = std::max(requested_max_accel, 1e-6f);
  elapsed = 0.0;

  if (distance < 1e-7) {
    // ya estamos en el destino, no hay nada que mover
    max_vel = 0.0f;
    t_accel = t_cruise = t_total = 0.0f;
    active = false;
    return;
  }

  // distancia recorrida acelerando desde 0 hasta requested_max_vel
  float accel_distance = 0.5f * requested_max_vel * requested_max_vel / max_accel;

  const float dist = float(distance);   // para los tiempos alcanza el float
  if (2.0f * accel_distance <= dist) {
    // perfil trapezoidal: alcanza velocidad de crucero
    max_vel = requested_max_vel;
    t_accel = max_vel / max_accel;
    float cruise_distance = dist - 2.0f * accel_distance;
    t_cruise = cruise_distance / max_vel;
  } else {
    // recorrido corto: perfil triangular, no llega a la velocidad pedida
    max_vel = std::sqrt(max_accel * dist);
    t_accel = max_vel / max_accel;
    t_cruise = 0.0f;
  }

  t_total = 2.0f * t_accel + t_cruise;
  active = true;
}

bool TrapezoidalProfile::update(float dt, double& out_pos, float& out_vel) {
  if (!active) {
    out_pos = current_pos = target_pos;
    out_vel = 0.0f;
    return false;
  }

  elapsed += dt;

  double pos_along_path = 0.0; // distancia recorrida desde start_pos, en [0, distance]
  float vel_along_path = 0.0f;

  if (elapsed >= t_total) {
    // movimiento terminado
    pos_along_path = distance;
    vel_along_path = 0.0f;
    active = false;
  } else if (elapsed < t_accel) {
    // rampa de aceleración
    float te = float(elapsed);
    pos_along_path = 0.5f * max_accel * te * te;
    vel_along_path = max_accel * te;
  } else if (elapsed < t_accel + t_cruise) {
    // tramo a velocidad constante
    float accel_distance = 0.5f * max_accel * t_accel * t_accel;
    double t_in_cruise = elapsed - t_accel;
    pos_along_path = accel_distance + double(max_vel) * t_in_cruise;
    vel_along_path = max_vel;
  } else {
    // rampa de desaceleración
    float t_in_decel = float(elapsed - t_accel - t_cruise);
    float accel_distance = 0.5f * max_accel * t_accel * t_accel;
    double cruise_distance = double(max_vel) * double(t_cruise);
    pos_along_path = accel_distance + cruise_distance +
                     (max_vel * t_in_decel - 0.5f * max_accel * t_in_decel * t_in_decel);
    vel_along_path = max_vel - max_accel * t_in_decel;
  }

  out_pos = current_pos = start_pos + double(direction) * pos_along_path;
  out_vel = direction * vel_along_path;
  return active;
}

double TrapezoidalProfile::get_current_position() const {
  return current_pos;
}

bool TrapezoidalProfile::is_idle() const {
  return !active;
}

double TrapezoidalProfile::get_target_position() const {
  return target_pos;
}
