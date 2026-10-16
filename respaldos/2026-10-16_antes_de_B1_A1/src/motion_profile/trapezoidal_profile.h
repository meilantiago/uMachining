// --------------------------------------------------------------------------------------
// Generador simple de perfil de velocidad trapezoidal (o triangular si no da el
// recorrido) para un solo eje lineal. Reemplaza al PathPlanner/MotionController
// multi-eje del proyecto original, que resolvía cinemática de un brazo delta y
// look-ahead entre múltiples segmentos: acá no hace falta nada de eso, es un
// movimiento punto-a-punto que arranca y termina con velocidad cero.
// --------------------------------------------------------------------------------------

#pragma once

//*** CLASS *****************************************************************************

class TrapezoidalProfile {
  public:
    // Arranca un nuevo movimiento desde 'start_pos' hasta 'target_pos' (en las
    // mismas unidades que max_vel/max_accel, típicamente mm y mm/s, mm/s^2).
    // Si ya había un movimiento en curso, lo reemplaza (arranca de cero, no
    // hace blending con la velocidad que tenía en ese momento).
    void start(float start_pos, float target_pos, float max_vel, float max_accel);

    // Avanza el perfil 'dt' segundos. Escribe la posición y velocidad objetivo
    // actuales en 'out_pos'/'out_vel'. Devuelve true mientras el movimiento
    // sigue en curso, false cuando ya llegó (out_pos queda fijo en target_pos).
    bool update(float dt, float& out_pos, float& out_vel);

    // true si no hay ningún movimiento en curso (recién construido o ya llegó)
    bool is_idle() const;

    // posición objetivo final del movimiento actual (o la última alcanzada)
    float get_target_position() const;

    // posición objetivo actual (la última que devolvió update(), o start_pos si todavía no corrió)
    float get_current_position() const;

  private:
    float current_pos = 0.0f;
    float start_pos = 0.0f;
    float target_pos = 0.0f;
    float direction = 1.0f;     // +1 o -1
    float distance = 0.0f;      // |target_pos - start_pos|

    float max_vel = 0.0f;       // velocidad de crucero efectiva (puede ser < max_vel pedido si el
                                 // recorrido es corto y el perfil resulta triangular)
    float max_accel = 0.0f;

    float t_accel = 0.0f;       // duración de la rampa de aceleración [s]
    float t_cruise = 0.0f;      // duración del tramo a velocidad constante [s]
    float t_total = 0.0f;       // duración total del movimiento [s]

    float elapsed = 0.0f;       // tiempo transcurrido desde el start() [s]
    bool active = false;
};
