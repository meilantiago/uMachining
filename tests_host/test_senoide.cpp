// --------------------------------------------------------------------------------------
// Tests de integración del comando M60 (senoide) que corren en la PC, sin la placa.
//
// Compilan los archivos reales del firmware que no dependen del hardware:
//   command_parser.cpp, sine_command.cpp, sine_profile.cpp, trapezoidal_profile.cpp,
//   setpoint.h (la misma composición trapecio + senoide que usa la interrupción).
// Una "máquina de PC" reemplaza a SingleAxisCommandProcessor: recibe texto por el mismo
// CommandParser, despacha G1 al perfil trapezoidal y M60 a execute_sine_command (el mismo
// código que llama el firmware), y avanza la consigna con ticks de ~500 µs con jitter,
// como el timer de 2 kHz.
//
// Uso:  test_senoide            corre los tests
//       test_senoide --csv f    además escribe el barrido de la prueba 4 en f (para el
//                               test de punta a punta en Python)
// --------------------------------------------------------------------------------------

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "hw_config.h"
#include "command_parser/command_parser.h"
#include "motion_profile/setpoint.h"
#include "sine_command.h"

//--- mini framework -------------------------------------------------------------------
static int g_fallas = 0, g_chequeos = 0;
static const char* g_test = "";

#define CHECK(cond, ...)                                                       \
  do {                                                                         \
    g_chequeos++;                                                              \
    if (!(cond)) {                                                             \
      g_fallas++;                                                              \
      std::printf("  FALLA [%s] %s:%d: %s -- ", g_test, __FILE__, __LINE__, #cond); \
      std::printf(__VA_ARGS__);                                                \
      std::printf("\n");                                                       \
    }                                                                          \
  } while (0)

static bool empieza(const std::string& s, const char* p) { return s.rfind(p, 0) == 0; }

//--- máquina de PC --------------------------------------------------------------------
struct Muestra {
  double t, pos, trap_pos;
  float vel;
  bool active, measuring;
  double freq;
};

class MaquinaPC : public ICommandProcessor {
  public:
    TrapezoidalProfile trap;
    SineProfile sine;
    bool ready = true;
    bool motor = true;
    std::string respuesta;
    double t = 0.0;
    uint32_t semilla = 12345;
    bool jitter = true;
    std::vector<Muestra> registro;

    MaquinaPC() {
      parser.set_command_processor(this);
      trap.start(30.0, 30.0, DEFAULT_MAX_VELOCITY_MM_S, DEFAULT_MAX_ACCEL_MM_S2);
    }

    void send_reply(const char* s) override { respuesta += s; }
    bool can_process_command(const GCodeCommand&) override { return true; }

    void process_command(const GCodeCommand& cmd, std::string& reply) override {
      const std::string& c = cmd.get_command();
      if (c == "G1") {
        double x = cmd.get_value_double('X', 0.0);
        float f = cmd.get_value('F', DEFAULT_MAX_VELOCITY_MM_S);
        trap.start(trap.get_current_position(), x, f, DEFAULT_MAX_ACCEL_MM_S2);
        reply = "ok\n";
        return;
      }
      if (c == "M60") {
        // mismo armado de contexto que SingleAxisCommandProcessor::process_sine_command
        SineContext ctx;
        ctx.ready = ready;
        ctx.motor_enabled = motor;
        double now = trap.get_current_position(), dest = trap.get_target_position();
        ctx.trap_speed_mm_s = trap.is_idle() ? 0.0 : trap.get_cruise_velocity();
        ctx.low_mm = std::fmin(now, dest);
        ctx.high_mm = std::fmax(now, dest);
        reply = execute_sine_command(cmd, ctx, sine, nullptr, nullptr);
        return;
      }
      reply = "error: unknown command '" + c + "'\n";
    }

    // Manda una línea por el parser real, como si llegara por el puerto serie.
    std::string enviar(const char* linea) {
      respuesta.clear();
      for (const char* p = linea; *p; ++p) parser.add_input_character(*p);
      parser.add_input_character('\n');
      parser.update();
      return respuesta;
    }

    // Un tick del timer de 2 kHz (500 µs ± 20 µs si hay jitter).
    void tick() {
      double dt = 500e-6;
      if (jitter) {
        semilla = semilla * 1664525u + 1013904223u;
        dt += ((semilla >> 8) / double(1u << 24) - 0.5) * 40e-6;
      }
      t += dt;
      Setpoint sp = update_setpoint(trap, sine, float(dt));
      registro.push_back({t, sp.pos_mm, trap.get_current_position(), sp.vel_mm_s, sp.sine_active,
                          sp.measuring, sine.is_active() ? sine.params().freq_hz : 0.0});
    }
    void correr(double segundos) {
      double fin = t + segundos;
      while (t < fin) tick();
    }
    void correr_hasta_inactiva(double max_s = 600.0) {
      double fin = t + max_s;
      while (sine.is_active() && t < fin) tick();
      tick();
    }

  private:
    static CommandParser parser_storage;   // estático: el parser no inicializa command_ready
    CommandParser& parser = parser_storage;
};
CommandParser MaquinaPC::parser_storage;

//--- utilidades de análisis -----------------------------------------------------------
struct Ajuste { double amp, fase, residuo_rms; };

// Ajuste por mínimos cuadrados de a·sen + b·cos + c a frecuencia conocida.
static Ajuste ajustar(const std::vector<Muestra>& r, size_t i0, size_t i1, double f, bool usar_trap = true) {
  double S[3][3] = {{0}}, B[3] = {0};
  for (size_t i = i0; i < i1; i++) {
    double w = 2 * M_PI * f * r[i].t;
    double y = r[i].pos - (usar_trap ? r[i].trap_pos : 0.0);
    double v[3] = {std::sin(w), std::cos(w), 1.0};
    for (int a = 0; a < 3; a++) {
      B[a] += v[a] * y;
      for (int b = 0; b < 3; b++) S[a][b] += v[a] * v[b];
    }
  }
  // Gauss 3x3
  for (int k = 0; k < 3; k++)
    for (int i = k + 1; i < 3; i++) {
      double m = S[i][k] / S[k][k];
      for (int j = k; j < 3; j++) S[i][j] -= m * S[k][j];
      B[i] -= m * B[k];
    }
  double x[3];
  for (int i = 2; i >= 0; i--) {
    x[i] = B[i];
    for (int j = i + 1; j < 3; j++) x[i] -= S[i][j] * x[j];
    x[i] /= S[i][i];
  }
  double ss = 0;
  for (size_t i = i0; i < i1; i++) {
    double w = 2 * M_PI * f * r[i].t;
    double y = r[i].pos - (usar_trap ? r[i].trap_pos : 0.0);
    double e = y - (x[0] * std::sin(w) + x[1] * std::cos(w) + x[2]);
    ss += e * e;
  }
  return {std::hypot(x[0], x[1]), std::atan2(x[1], x[0]), std::sqrt(ss / double(i1 - i0))};
}

static void ventana(const std::vector<Muestra>& r, size_t& i0, size_t& i1) {
  i0 = i1 = 0;
  for (size_t i = 0; i < r.size(); i++)
    if (r[i].measuring) { i0 = i; break; }
  for (size_t i = i0; i < r.size(); i++)
    if (!r[i].measuring) { i1 = i; break; }
}

//--- tests ----------------------------------------------------------------------------
static void test_interpretacion() {
  g_test = "interpretacion";
  MaquinaPC m;
  CHECK(empieza(m.enviar("M60"), "senoide inactiva"), "%s", m.respuesta.c_str());
  std::string r = m.enviar("M60 A1 F50 N10");
  CHECK(empieza(r, "senoide A=1.0000 um F=50.000 Hz N=10"), "%s", r.c_str());
  CHECK(r.find("ok\n") != std::string::npos, "%s", r.c_str());
  CHECK(m.sine.is_active(), "no arranco");
  CHECK(empieza(m.enviar("M60 A1 F20 N5"), "error: ya hay una senoide en curso"), "%s", m.respuesta.c_str());
  CHECK(m.enviar("M60 S0") == "ok\n", "%s", m.respuesta.c_str());

  struct { const char* cmd; const char* esperado; } casos[] = {
    {"M60 A1 F50", "error: M60 necesita"},
    {"M60 A0 F50 N10", "error: amplitud fuera de rango"},
    {"M60 A60 F1 N10", "error: amplitud fuera de rango"},
    {"M60 A1 F300 N10", "error: frecuencia fuera de rango"},
    {"M60 A1 F0.05 N10", "error: frecuencia fuera de rango"},
    {"M60 A1 F50 N2.5", "error: N tiene que ser un entero"},
    {"M60 A1 F50 N0", "error: N tiene que ser un entero"},
    {"M60 A1 F50 N10 R-1", "error: R y T"},
    {"M60 A1 F50 N10 X3", "error: M60 acepta"},
    {"M60 S1", "error: M60 S solo acepta S0"},
    {"M60 A20 F100 N10", "error: velocidad pico"},      // 2π·100·20 µm = 12,6 mm/s
  };
  MaquinaPC q;
  for (auto& c : casos) {
    std::string rr = q.enviar(c.cmd);
    CHECK(empieza(rr, c.esperado), "%s -> %s", c.cmd, rr.c_str());
    CHECK(!q.sine.is_active(), "%s arranco igual", c.cmd);
  }

  MaquinaPC sin_home;
  sin_home.ready = false;
  CHECK(empieza(sin_home.enviar("M60 A1 F50 N10"), "error: eje no homeado"), "%s", sin_home.respuesta.c_str());
  MaquinaPC sin_motor;
  sin_motor.motor = false;
  CHECK(empieza(sin_motor.enviar("M60 A1 F50 N10"), "error: motor deshabilitado"), "%s", sin_motor.respuesta.c_str());

  // Cerca del tope: 0,5 µm del cero con 1 µm de amplitud se sale del recorrido
  MaquinaPC tope;
  tope.trap.start(0.0005, 0.0005, 2.0f, 5.0f);
  CHECK(empieza(tope.enviar("M60 A1 F50 N10"), "error: la senoide se sale"), "%s", tope.respuesta.c_str());

  // Sobre un G1 a 4,5 mm/s quedan 0,5 mm/s: 1 µm a 100 Hz (0,63 mm/s) no entra
  MaquinaPC rapida;
  rapida.enviar("G1 X60 F4.5");
  rapida.correr(1.5);
  CHECK(empieza(rapida.enviar("M60 A1 F100 N10"), "error: velocidad pico"), "%s", rapida.respuesta.c_str());
}

static void test_tiempos() {
  g_test = "tiempos";
  SineProfile s;
  SineParams p;
  p.amplitude_mm = 1e-3; p.freq_hz = 50; p.cycles = 10; p.ramp_s = 0.2; p.settle_s = 0.3;
  CHECK(s.start(p), "start");
  CHECK(std::fabs(s.ramp_in_s() - 0.2) < 1e-12, "entrada %.6f", s.ramp_in_s());
  CHECK(std::fabs(s.settle_time_s() - 0.3) < 1e-12, "asentamiento %.6f", s.settle_time_s());
  CHECK(std::fabs(s.measure_time_s() - 0.2) < 1e-12, "medicion %.6f", s.measure_time_s());
  CHECK(std::fabs(s.total_time_s() - 0.9) < 1e-12, "total %.6f", s.total_time_s());

  p.freq_hz = 7; p.cycles = 3;   // 0,2 s -> 2 ciclos; 0,3 s -> 3 ciclos (2,1 redondea arriba)
  s.start(p);
  CHECK(std::fabs(s.ramp_in_s() - 2.0 / 7) < 1e-12, "entrada %.6f", s.ramp_in_s());
  CHECK(std::fabs(s.settle_time_s() - 3.0 / 7) < 1e-12, "asentamiento %.6f", s.settle_time_s());

  p.freq_hz = 1; p.ramp_s = 0.0; p.settle_s = 0.0;   // la entrada nunca baja de 1 ciclo
  s.start(p);
  CHECK(std::fabs(s.ramp_in_s() - 1.0) < 1e-12, "entrada %.6f", s.ramp_in_s());
  CHECK(s.settle_time_s() == 0.0, "asentamiento %.6f", s.settle_time_s());

  SineParams malo = p;
  malo.cycles = 0;
  CHECK(!s.start(malo), "acepto 0 ciclos");
}

// Corre una senoide en la máquina de PC y verifica la forma de la consigna.
static void verificar_senoide(MaquinaPC& m, double A_um, double f, int N, const char* nombre) {
  size_t i0, i1;
  ventana(m.registro, i0, i1);
  const auto& r = m.registro;
  CHECK(i1 > i0, "%s: no hubo ventana de medicion", nombre);
  if (i1 <= i0) return;

  // ventana = N ciclos (con la resolucion de un tick)
  double dur = r[i1].t - r[i0].t;
  CHECK(std::fabs(dur - N / f) < 0.6e-3, "%s: ventana %.6f s, esperado %.6f s", nombre, dur, N / f);

  // la ventana arranca con fase 0: el desvio vale A·sen(ω·δt) con δt < 1 tick
  double off0 = r[i0].pos - r[i0].trap_pos;
  double w = 2 * M_PI * f;
  CHECK(off0 >= -1e-12 && off0 <= A_um * 1e-3 * std::sin(std::fmin(w * 0.55e-3, M_PI / 2)) + 1e-12,
        "%s: desvio al abrir la ventana %.3g nm", nombre, off0 * 1e6);

  // amplitud y forma dentro de la ventana
  Ajuste a = ajustar(r, i0, i1, f);
  CHECK(std::fabs(a.amp / (A_um * 1e-3) - 1) < 1e-4, "%s: amplitud %.6f um", nombre, a.amp * 1e3);
  CHECK(a.residuo_rms < 1e-3 * A_um * 1e-3, "%s: residuo %.3g nm", nombre, a.residuo_rms * 1e6);

  // sin saltos: ningun paso supera la velocidad pico por el tick mas largo
  SineParams p = m.sine.params();
  double vmax = SineProfile::peak_velocity(p) + 5.0;   // + margen por el trapecio
  double salto = 0, err_v = 0, vref = 2 * M_PI * f * A_um * 1e-3;
  for (size_t i = 1; i + 1 < r.size(); i++) {
    double d = std::fabs((r[i].pos - r[i].trap_pos) - (r[i - 1].pos - r[i - 1].trap_pos));
    salto = std::fmax(salto, d / (r[i].t - r[i - 1].t));
  }
  CHECK(salto <= vmax, "%s: salto equivalente a %.2f mm/s", nombre, salto);

  // la velocidad para la realimentacion anticipada coincide con la derivada de la posicion
  for (size_t i = 1; i + 1 < r.size(); i++) {
    if (!r[i - 1].active || !r[i + 1].active) continue;
    double d = (r[i + 1].pos - r[i - 1].pos) / (r[i + 1].t - r[i - 1].t);
    err_v = std::fmax(err_v, std::fabs(d - r[i].vel));
  }
  // la diferencia centrada con ticks de 0,5 ms se aparta de la derivada en ~(ωdt)²/6, y el
  // jitter (ticks desiguales) agrega ~ω·20 µs
  double wdt = w * 0.52e-3;
  double tol_v = vref * (1.2 * wdt * wdt / 6 + w * 25e-6 + 0.005) + 1e-6;
  CHECK(err_v < tol_v, "%s: velocidad vs derivada %.4f mm/s (tolerancia %.4f)", nombre, err_v, tol_v);

  // al terminar: inactiva, desvio exactamente cero
  const Muestra& fin = r.back();
  CHECK(!fin.active && !fin.measuring, "%s: sigue activa al final", nombre);
  CHECK(fin.pos == fin.trap_pos, "%s: desvio final %.3g nm", nombre, (fin.pos - fin.trap_pos) * 1e6);
}

static void test_senoide_quieto() {
  g_test = "senoide_quieto";
  const double fs[] = {1, 10, 50, 100, 200, 250};
  for (double f : fs) {
    MaquinaPC m;
    char cmd[64];
    std::snprintf(cmd, sizeof(cmd), "M60 A1 F%g N10", f);
    std::string r = m.enviar(cmd);
    CHECK(empieza(r, "senoide"), "%s -> %s", cmd, r.c_str());
    m.correr_hasta_inactiva();
    char nombre[32];
    std::snprintf(nombre, sizeof(nombre), "%g Hz", f);
    verificar_senoide(m, 1.0, f, 10, nombre);
    // el trapecio no se movio
    CHECK(m.registro.back().trap_pos == 30.0, "%s: trapecio en %.9f", nombre, m.registro.back().trap_pos);
  }
  // 100 nm
  MaquinaPC m;
  m.enviar("M60 A0.1 F70 N20");
  m.correr_hasta_inactiva();
  verificar_senoide(m, 0.1, 70, 20, "100 nm 70 Hz");
}

static void test_senoide_sobre_rampa() {
  g_test = "senoide_sobre_rampa";
  MaquinaPC m;
  m.enviar("G1 X80 F2");                 // 50 mm a 2 mm/s
  m.correr(1.0);                          // ya en crucero
  std::string r = m.enviar("M60 A1 F130 N50");
  CHECK(empieza(r, "senoide"), "%s", r.c_str());
  m.correr_hasta_inactiva();
  verificar_senoide(m, 1.0, 130, 50, "130 Hz sobre rampa");
  // la consigna menos el trapecio es la senoide sola; el trapecio sigue su curso
  CHECK(!m.trap.is_idle(), "el G1 termino antes de tiempo");
  m.correr(30.0);
  CHECK(std::fabs(m.registro.back().pos - 80.0) < 1e-9, "no llego a X80: %.9f", m.registro.back().pos);
}

static void test_frenado() {
  g_test = "frenado";
  MaquinaPC m;
  m.enviar("M60 A2 F20 N200");
  m.correr(1.0);                          // en plena medicion
  CHECK(m.sine.is_measuring(), "no estaba midiendo");
  size_t antes = m.registro.size();
  CHECK(m.enviar("M60 S0") == "ok\n", "%s", m.respuesta.c_str());
  CHECK(!m.sine.is_measuring() && m.sine.is_stopping(), "no empezo a frenar");
  CHECK(empieza(m.enviar("M60"), "senoide frenando"), "%s", m.respuesta.c_str());
  double t0 = m.t;
  m.correr_hasta_inactiva();
  double t_freno = m.t - t0;
  CHECK(t_freno <= m.sine.ramp_in_s() + 1e-3, "tardo %.3f s en frenar", t_freno);
  // sin saltos al frenar
  double salto = 0;
  for (size_t i = antes; i < m.registro.size(); i++)
    salto = std::fmax(salto, std::fabs(m.registro[i].pos - m.registro[i - 1].pos));
  double max_paso = SineProfile::peak_velocity(m.sine.params()) * 0.52e-3;
  CHECK(salto <= max_paso, "salto de %.3g nm (max %.3g)", salto * 1e6, max_paso * 1e6);
  CHECK(m.registro.back().pos == 30.0, "no volvio al centro: %.9f", m.registro.back().pos);
  CHECK(empieza(m.enviar("M60"), "senoide inactiva segmentos=1"), "%s", m.respuesta.c_str());
}

static void test_abort() {
  g_test = "abort";
  SineProfile s;
  SineParams p;
  p.amplitude_mm = 1e-3; p.freq_hz = 50; p.cycles = 10;
  s.start(p);
  double off; float v;
  for (int i = 0; i < 1000; i++) s.update(500e-6, off, v);
  s.abort();
  bool activa = s.update(500e-6, off, v);
  CHECK(!activa && off == 0.0 && v == 0.0f && !s.is_measuring(), "abort no corto");
}

// Barrido de la prueba 4: 13 frecuencias, una tras otra, esperando que termine cada una.
static const double FRECUENCIAS[] = {1, 2, 5, 10, 20, 30, 50, 70, 100, 130, 150, 170, 200};

static void barrido(MaquinaPC& m) {
  for (double f : FRECUENCIAS) {
    int n = int(std::ceil(std::fmax(5.0, 0.5 * f)));
    char cmd[64];
    std::snprintf(cmd, sizeof(cmd), "M60 A1 F%g N%d", f, n);
    std::string r = m.enviar(cmd);
    CHECK(empieza(r, "senoide"), "%s -> %s", cmd, r.c_str());
    m.correr_hasta_inactiva();
    m.correr(0.1);
  }
}

static void test_barrido() {
  g_test = "barrido";
  MaquinaPC m;
  m.correr(0.5);
  barrido(m);
  // contar ventanas y verificar cada una
  int ventanas = 0;
  const auto& r = m.registro;
  for (size_t i = 1; i < r.size(); i++) {
    if (r[i].measuring && !r[i - 1].measuring) {
      size_t j = i;
      while (j < r.size() && r[j].measuring) j++;
      double f = r[i].freq;
      Ajuste a = ajustar(r, i, j, f);
      CHECK(std::fabs(a.amp - 1e-3) < 1e-7, "%g Hz: amplitud %.6f um", f, a.amp * 1e3);
      CHECK(f == FRECUENCIAS[ventanas], "ventana %d en %g Hz", ventanas, f);
      ventanas++;
    }
  }
  CHECK(ventanas == 13, "%d ventanas", ventanas);
  CHECK(m.sine.segment_count() == 13, "%u segmentos", m.sine.segment_count());
  std::printf("  barrido: %.1f s de consigna, %zu ticks\n", m.t, r.size());
}

static int escribir_csv(const char* ruta) {
  MaquinaPC m;
  m.jitter = false;            // la simulación de Python espera 2 kHz exactos
  m.correr(0.5);
  barrido(m);
  m.correr(0.5);
  FILE* f = std::fopen(ruta, "w");
  if (!f) return 1;
  std::fprintf(f, "t_s,pos_mm,vel_mm_s,midiendo,activa,f_hz\n");
  for (const auto& s : m.registro)
    std::fprintf(f, "%.6f,%.12f,%.9g,%d,%d,%g\n", s.t, s.pos, double(s.vel), int(s.measuring), int(s.active),
                 s.freq);
  std::fclose(f);
  std::printf("  csv: %zu filas en %s\n", m.registro.size(), ruta);
  return 0;
}

int main(int argc, char** argv) {
  test_interpretacion();
  test_tiempos();
  test_senoide_quieto();
  test_senoide_sobre_rampa();
  test_frenado();
  test_abort();
  test_barrido();
  if (argc == 3 && std::strcmp(argv[1], "--csv") == 0 && escribir_csv(argv[2]) != 0) {
    std::printf("no se pudo escribir %s\n", argv[2]);
    return 2;
  }
  std::printf("%d chequeos, %d fallas\n", g_chequeos, g_fallas);
  return g_fallas == 0 ? 0 : 1;
}
