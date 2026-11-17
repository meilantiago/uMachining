// --------------------------------------------------------------------------------------
// Consigna que se le pasa al lazo en cada tick de 2 kHz: perfil trapezoidal + senoide.
// Está en un archivo aparte, sin dependencias del hardware, para que la interrupción del
// firmware (main.cpp) y los tests de PC (tests_host/) usen exactamente el mismo código.
// --------------------------------------------------------------------------------------

#pragma once

#include "trapezoidal_profile.h"
#include "sine_profile.h"

struct Setpoint {
  double pos_mm = 0.0;       // posición comandada (trapecio + senoide)
  float  vel_mm_s = 0.0f;    // velocidad comandada, para la realimentación anticipada
  bool   sine_active = false;
  bool   measuring = false;  // ventana de medición de la senoide (pin de sincronismo y telemetría)
};

inline Setpoint update_setpoint(TrapezoidalProfile& trap, SineProfile& sine, float dt) {
  Setpoint s;
  trap.update(dt, s.pos_mm, s.vel_mm_s);
  double off_mm;
  float off_vel;
  s.sine_active = sine.update(dt, off_mm, off_vel);
  s.pos_mm += off_mm;
  s.vel_mm_s += off_vel;
  s.measuring = sine.is_measuring();
  return s;
}
