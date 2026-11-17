#include "sine_profile.h"
#include <cmath>

namespace {
constexpr double PI = 3.14159265358979323846;
}

double SineProfile::round_to_cycles(double t_s, double freq_hz, int min_cycles) {
  double n = std::ceil(t_s * freq_hz - 1e-9);
  if (n < min_cycles) n = min_cycles;
  return n / freq_hz;
}

double SineProfile::peak_velocity(const SineParams& p) {
  // |d/dt (A e sen ωt)| <= A (|e'| + ω), con |e'| <= π / (2 t_entrada)
  double w = 2.0 * PI * p.freq_hz;
  double t_in = round_to_cycles(p.ramp_s, p.freq_hz, 1);
  return p.amplitude_mm * (w + PI / (2.0 * t_in));
}

double SineProfile::peak_acceleration(const SineParams& p) {
  // A (|e''| + 2|e'|ω + ω²), con |e''| <= π² / (2 t²)
  double w = 2.0 * PI * p.freq_hz;
  double t_in = round_to_cycles(p.ramp_s, p.freq_hz, 1);
  double e1 = PI / (2.0 * t_in);
  double e2 = PI * PI / (2.0 * t_in * t_in);
  return p.amplitude_mm * (e2 + 2.0 * e1 * w + w * w);
}

bool SineProfile::start(const SineParams& np) {
  if (!(np.amplitude_mm > 0.0) || !(np.freq_hz > 0.0) || np.cycles < 1 ||
      np.ramp_s < 0.0 || np.settle_s < 0.0 || !std::isfinite(np.amplitude_mm) || !std::isfinite(np.freq_hz))
    return false;
  p = np;
  omega = 2.0 * PI * p.freq_hz;
  t_in = round_to_cycles(p.ramp_s, p.freq_hz, 1);
  t_settle = round_to_cycles(p.settle_s, p.freq_hz, 0);
  t_meas = p.cycles / p.freq_hz;
  t_out = t_in;
  elapsed = 0.0;
  out_from = 1.0;
  out_start = 0.0;
  measuring = false;
  stopped_early = false;
  state = State::RAMP_IN;
  segments++;
  return true;
}

void SineProfile::stop() {
  if (state == State::IDLE || state == State::RAMP_OUT) return;
  double e = 1.0;
  if (state == State::RAMP_IN) e = 0.5 - 0.5 * std::cos(PI * elapsed / t_in);
  out_from = e;
  out_start = elapsed;
  state = State::RAMP_OUT;
  stopped_early = true;
  measuring = false;
}

void SineProfile::abort() {
  state = State::IDLE;
  measuring = false;
}

bool SineProfile::update(double dt, double& offset_mm, float& vel_mm_s) {
  offset_mm = 0.0;
  vel_mm_s = 0.0f;
  if (state == State::IDLE) return false;

  elapsed += dt;
  const double t = elapsed;

  // fin de la entrada y comienzo de la salida programada
  if (state == State::RAMP_IN && t >= t_in) state = State::HOLD;
  if (state == State::HOLD && t >= t_in + t_settle + t_meas) {
    state = State::RAMP_OUT;
    out_start = t_in + t_settle + t_meas;
    out_from = 1.0;
  }

  double e = 1.0, de = 0.0;
  switch (state) {
    case State::RAMP_IN:
      e = 0.5 - 0.5 * std::cos(PI * t / t_in);
      de = 0.5 * PI / t_in * std::sin(PI * t / t_in);
      break;
    case State::HOLD:
      break;
    case State::RAMP_OUT: {
      double tau = t - out_start;
      if (tau >= t_out) {
        state = State::IDLE;
        measuring = false;
        return false;
      }
      e = out_from * (0.5 + 0.5 * std::cos(PI * tau / t_out));
      de = -out_from * 0.5 * PI / t_out * std::sin(PI * tau / t_out);
      break;
    }
    case State::IDLE:
      return false;
  }

  measuring = (state == State::HOLD) && t >= t_in + t_settle && t < t_in + t_settle + t_meas;

  const double s = std::sin(omega * t);
  const double c = std::cos(omega * t);
  offset_mm = p.amplitude_mm * e * s;
  vel_mm_s = float(p.amplitude_mm * (de * s + e * omega * c));
  return true;
}
