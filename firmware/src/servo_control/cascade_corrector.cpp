#include "cascade_corrector.h"
#include <algorithm>  // std::min, std::max

void LinearCascadeCorrector::set_gains(float kp, float ki, float integral_limit) {
  kp_ = kp;
  ki_ = ki;
  integral_limit_ = integral_limit;
}

float LinearCascadeCorrector::update(float target_pos, float linear_pos, float dt) {
  float error = target_pos - linear_pos;

  // Integrar con anti-windup: clampar el acumulador para evitar saturación
  // tras arranques, perturbaciones grandes, o movimientos largos con error sostenido.
  integral_ += error * dt;
  integral_ = std::max(-integral_limit_, std::min(integral_, integral_limit_));

  // Salida = referencia + corrección proporcional + corrección integral
  return target_pos + kp_ * error + ki_ * integral_;
}

void LinearCascadeCorrector::reset() {
  integral_ = 0.0f;
}
