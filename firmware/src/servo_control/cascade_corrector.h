// --------------------------------------------------------------------------------------
// LinearCascadeCorrector: controlador PI lento para lazo externo de cascada.
// Compara la posición comandada (salida del perfil trapezoidal) con la lectura
// de un encoder lineal externo montado en el carro, y genera una corrección
// aditiva para compensar backlash, derivas térmicas, flexiones mecánicas, etc.
//
// Topología en el firmware:
//   TrapezoidalProfile.update() → pos_mm (referencia)
//   read_linear_encoder()       → linear_pos_mm (medición)
//   LinearCascadeCorrector.update(pos_mm, linear_pos_mm, dt) → corrected_pos_mm
//   SharedData.target_position_mm = corrected_pos_mm
//
// Fórmula:
//   Error  = target_pos - linear_pos
//   Output = target_pos + Kp * Error + Ki * Integral(Error)
//
// El integrador tiene anti-windup por clamp para evitar saturación al arranque
// o tras una perturbación grande. Llamar a reset() al homear o calibrar.
// --------------------------------------------------------------------------------------

#pragma once

class LinearCascadeCorrector {
  public:
    LinearCascadeCorrector() = default;

    // Configura las ganancias del controlador PI y el límite de anti-windup.
    //   kp:              ganancia proporcional
    //   ki:              ganancia integral
    //   integral_limit:  clamp simétrico del acumulador integral [mm] (anti-windup)
    void set_gains(float kp, float ki, float integral_limit);

    // Calcula la posición corregida.
    //   target_pos:  posición objetivo actual [mm] (salida del perfil trapezoidal)
    //   linear_pos:  posición real del encoder lineal [mm]
    //   dt:          tiempo desde la última llamada [s]
    //   Retorna:     posición corregida [mm] (target + corrección PI)
    float update(float target_pos, float linear_pos, float dt);

    // Resetea el acumulador integral a cero.
    // Llamar antes de homing (G28) y calibración (M56) para evitar windup residual.
    void reset();

  private:
    float kp_ = 0.5f;              // ganancia proporcional (conservadora por defecto)
    float ki_ = 2.0f;              // ganancia integral
    float integral_limit_ = 1.0f;  // ±mm, clamp anti-windup

    float integral_ = 0.0f;        // acumulador integral [mm·s → convertido a mm por ki]
};
