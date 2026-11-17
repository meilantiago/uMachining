#include "sine_command.h"
#include "hw_config.h"

#include <cmath>
#include <cstdio>

static SineAction error(const char* msg) {
  SineAction a;
  a.kind = SineAction::Kind::ERROR;
  a.message = std::string("error: ") + msg + "\n";
  return a;
}

SineAction parse_sine_command(const GCodeCommand& cmd, const SineContext& ctx) {
  if (cmd.contains_unsupported_words("AFNRTS"))
    return error("M60 acepta A<um> F<Hz> N<ciclos> [R<s>] [T<s>] o S0");

  if (cmd.has_word('S')) {
    if (cmd.get_value('S') != 0.0f) return error("M60 S solo acepta S0 (frenar)");
    SineAction a;
    a.kind = SineAction::Kind::STOP;
    return a;
  }

  if (!cmd.has_word('A') && !cmd.has_word('F') && !cmd.has_word('N')) {
    SineAction a;
    a.kind = SineAction::Kind::STATUS;
    return a;
  }

  if (!cmd.has_word('A') || !cmd.has_word('F') || !cmd.has_word('N'))
    return error("M60 necesita A<um>, F<Hz> y N<ciclos>");
  if (ctx.sine_active) return error("ya hay una senoide en curso (M60 S0 para frenarla)");
  if (!ctx.ready) return error("eje no homeado/calibrado, correr G28 y M56 primero");
  if (!ctx.motor_enabled) return error("motor deshabilitado, mandar M17");

  SineParams p;
  const double a_um = cmd.get_value_double('A', 0.0);
  p.amplitude_mm = a_um * 1e-3;
  p.freq_hz = cmd.get_value_double('F', 0.0);
  const double n = cmd.get_value_double('N', 0.0);
  p.ramp_s = cmd.get_value_double('R', SINE_DEFAULT_RAMP_S);
  p.settle_s = cmd.get_value_double('T', SINE_DEFAULT_SETTLE_S);

  char buf[128];
  if (!(a_um > 0.0) || a_um > SINE_MAX_AMPLITUDE_UM) {
    snprintf(buf, sizeof(buf), "amplitud fuera de rango (0, %.1f] um", double(SINE_MAX_AMPLITUDE_UM));
    return error(buf);
  }
  if (!(p.freq_hz >= SINE_MIN_FREQ_HZ) || p.freq_hz > SINE_MAX_FREQ_HZ) {
    snprintf(buf, sizeof(buf), "frecuencia fuera de rango [%.2f, %.1f] Hz", double(SINE_MIN_FREQ_HZ),
             double(SINE_MAX_FREQ_HZ));
    return error(buf);
  }
  if (!(n >= 1.0) || n > SINE_MAX_CYCLES || n != std::floor(n)) {
    snprintf(buf, sizeof(buf), "N tiene que ser un entero entre 1 y %d", SINE_MAX_CYCLES);
    return error(buf);
  }
  p.cycles = int(n);
  if (!(p.ramp_s >= 0.0) || p.ramp_s > SINE_MAX_RAMP_S || !(p.settle_s >= 0.0) || p.settle_s > SINE_MAX_RAMP_S) {
    snprintf(buf, sizeof(buf), "R y T tienen que estar entre 0 y %.0f s", double(SINE_MAX_RAMP_S));
    return error(buf);
  }

  const double v_disp = MAX_VELOCITY_LIMIT_MM_S - std::fabs(ctx.trap_speed_mm_s);
  const double v_pico = SineProfile::peak_velocity(p);
  if (v_pico > v_disp) {
    snprintf(buf, sizeof(buf), "velocidad pico %.2f mm/s: supera el limite (quedan %.2f mm/s)", v_pico, v_disp);
    return error(buf);
  }
  const double a_pico = SineProfile::peak_acceleration(p);
  if (a_pico > SINE_MAX_ACCEL_MM_S2) {
    snprintf(buf, sizeof(buf), "aceleracion pico %.0f mm/s2: supera %.0f mm/s2", a_pico,
             double(SINE_MAX_ACCEL_MM_S2));
    return error(buf);
  }
  if (ctx.low_mm - p.amplitude_mm < AXIS_MIN_POSITION_MM || ctx.high_mm + p.amplitude_mm > AXIS_MAX_POSITION_MM)
    return error("la senoide se sale del recorrido");

  SineAction a;
  a.kind = SineAction::Kind::START;
  a.params = p;
  return a;
}

std::string execute_sine_command(const GCodeCommand& cmd, SineContext ctx, SineProfile& sine,
                                 uint32_t (*lock)(), void (*unlock)(uint32_t)) {
  ctx.sine_active = sine.is_active();
  SineAction a = parse_sine_command(cmd, ctx);
  uint32_t estado = 0;
  switch (a.kind) {
    case SineAction::Kind::ERROR:
      return a.message;
    case SineAction::Kind::STATUS:
      return sine_status_reply(sine);
    case SineAction::Kind::STOP:
      if (lock) estado = lock();
      sine.stop();
      if (unlock) unlock(estado);
      return "ok\n";
    case SineAction::Kind::START: {
      if (lock) estado = lock();
      bool ok = sine.start(a.params);
      if (unlock) unlock(estado);
      return ok ? sine_start_reply(sine) : "error: parametros invalidos\n";
    }
  }
  return "error: M60\n";
}

std::string sine_start_reply(const SineProfile& s) {
  char buf[200];
  snprintf(buf, sizeof(buf),
           "senoide A=%.4f um F=%.3f Hz N=%d entrada=%.4f s asentamiento=%.4f s "
           "medicion=%.4f..%.4f s total=%.4f s\nok\n",
           s.params().amplitude_mm * 1e3, s.params().freq_hz, s.params().cycles, s.ramp_in_s(),
           s.settle_time_s(), s.measure_start_s(), s.measure_end_s(), s.total_time_s());
  return buf;
}

std::string sine_status_reply(const SineProfile& s) {
  char buf[200];
  if (!s.is_active()) {
    snprintf(buf, sizeof(buf), "senoide inactiva segmentos=%lu\nok\n", (unsigned long)s.segment_count());
  } else {
    const char* fase = s.is_stopping() ? "frenando" : (s.is_measuring() ? "midiendo" : "activa");
    snprintf(buf, sizeof(buf), "senoide %s A=%.4f um F=%.3f Hz t=%.3f de %.3f s segmento=%lu\nok\n", fase,
             s.params().amplitude_mm * 1e3, s.params().freq_hz, s.elapsed_s(), s.total_time_s(),
             (unsigned long)s.segment_count());
  }
  return buf;
}
