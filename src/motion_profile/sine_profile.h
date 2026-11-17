// --------------------------------------------------------------------------------------
// Generador de senoide para la prueba de respuesta en frecuencia (prueba 4, ver
// docs/PRUEBAS_DINAMICAS_INTERFEROMETRO.md y simulaciones/prueba4/RESULTADOS.md).
//
// Produce un desvío x(t) = A·e(t)·sen(2πft) y su velocidad, que se SUMAN a la salida del
// perfil trapezoidal (ver setpoint.h). Así la senoide sirve con el carro quieto o sobre
// una rampa de G1 (para medir con la fricción deslizando).
//
// Envolvente e(t), con todos los tramos redondeados a ciclos enteros:
//   entrada  : coseno alzado de 0 a 1 (t_entrada, al menos 1 ciclo)
//   asentar  : amplitud plena, sin medir (t_asentar, puede ser 0)
//   medición : amplitud plena, N ciclos exactos; is_measuring() = true
//   salida   : coseno alzado de 1 a 0 (igual que la entrada)
// Como la entrada y el asentamiento son ciclos enteros, la ventana de medición empieza
// con fase 0 (sen = 0, subiendo).
//
// No depende del hardware: se compila igual en el RP2040 y en los tests de PC.
// --------------------------------------------------------------------------------------

#pragma once

#include <cstdint>

struct SineParams {
  double amplitude_mm = 0.0;   // amplitud A [mm]
  double freq_hz = 0.0;        // frecuencia f [Hz]
  int    cycles = 0;           // ciclos en la ventana de medición
  double ramp_s = 0.2;         // duración pedida de la entrada y de la salida [s]
  double settle_s = 0.3;       // duración pedida del asentamiento [s]
};

class SineProfile {
  public:
    // Arranca una senoide nueva (reemplaza la anterior sin transición: validar y frenar
    // antes si hacía falta). Devuelve false si los parámetros no tienen sentido.
    bool start(const SineParams& p);

    // Frena suave: sale con un coseno alzado desde la amplitud actual, sin cortar la fase.
    // La ventana de medición termina en ese momento.
    void stop();

    // Corta en seco (desvío 0). Solo para homing, calibración, motor apagado o falla.
    void abort();

    // Avanza dt segundos. Escribe el desvío [mm] y su velocidad [mm/s]. Devuelve true
    // mientras la senoide está activa; al terminar escribe 0 y devuelve false.
    bool update(double dt, double& offset_mm, float& vel_mm_s);

    bool is_active() const { return state != State::IDLE; }
    bool is_measuring() const { return measuring; }
    bool is_stopping() const { return state == State::RAMP_OUT && stopped_early; }

    const SineParams& params() const { return p; }
    double elapsed_s() const { return elapsed; }
    double ramp_in_s() const { return t_in; }
    double settle_time_s() const { return t_settle; }
    double measure_time_s() const { return t_meas; }
    double measure_start_s() const { return t_in + t_settle; }
    double measure_end_s() const { return t_in + t_settle + t_meas; }
    double total_time_s() const { return t_in + t_settle + t_meas + t_out; }
    uint32_t segment_count() const { return segments; }   // senoides arrancadas desde el inicio

    // Cotas de la velocidad y la aceleración pico, incluida la envolvente.
    static double peak_velocity(const SineParams& p);
    static double peak_acceleration(const SineParams& p);
    // Duración de la entrada/asentamiento redondeada a ciclos enteros.
    static double round_to_cycles(double t_s, double freq_hz, int min_cycles);

  private:
    enum class State { IDLE, RAMP_IN, HOLD, RAMP_OUT };

    SineParams p;
    State state = State::IDLE;
    double omega = 0.0;
    double t_in = 0.0, t_settle = 0.0, t_meas = 0.0, t_out = 0.0;
    double elapsed = 0.0;
    double out_start = 0.0;       // instante en que empezó la salida
    double out_from = 1.0;        // envolvente al empezar la salida
    bool measuring = false;
    bool stopped_early = false;
    uint32_t segments = 0;
};
