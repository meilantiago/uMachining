// --------------------------------------------------------------------------------------
// Datos compartidos entre los dos núcleos del RP2040/RP2350:
//  - Core 0 corre el perfil trapezoidal (2 kHz) y escribe la posición objetivo actual.
//  - Core 1 corre el lazo servo (lo más rápido posible) y lee esa posición objetivo.
// Mismo patrón que 'SharedData' en robot.h del proyecto original, pero para 1 solo eje.
//
// Telemetría: un ring buffer estático de TelemetryPoint permite a la ISR de 2 kHz
// empaquetar el estado del eje sin bloquear, y loop() lo drena como stream binario.
// --------------------------------------------------------------------------------------

#pragma once

#include <cstdint>
#include "pico/sync.h"
#include "servo_control/cogging_learner.h"

//*** TELEMETRY *************************************************************************

// 16/10/2026: se agregaron vel, out, chip_raw, status, flags y seq (antes 16 bytes, con
// linear_pos_mm que repetia motor_pos_mm). Paquete: 0xAA 0x55 + 28 bytes, little endian.
// Lo lee tools/closed_loop_test.py.
struct TelemetryPoint {
  uint32_t timestamp_us;   // time_us_64() truncado a 32 bits (wraps ~71 min)
  float    target_mm;      // posición objetivo (salida del perfil trapezoidal)
  float    motor_pos_mm;   // posición del motor (última lectura de encoder, cacheada)
  float    vel;            // velocidad del motor [rad/s]
  float    out;            // salida del lazo de velocidad [rad electricos], limite +-0.45*pi
  int32_t  chip_raw;       // angulo crudo del MT6835 (0..2^21-1)
  uint8_t  status;         // bits de estado del encoder (overspeed, campo debil, subtension)
  uint8_t  flags;          // bit0 motor_update (lazo activo), bit1 corte por falla
  uint16_t seq;            // contador de puntos generados (los huecos = puntos descartados)
};
static_assert(sizeof(TelemetryPoint) == 28, "el script de PC espera 28 bytes");

// Cola circular estática, sin asignación dinámica de memoria.
// El productor (ISR) usa spin_try_lock_unsafe (no bloqueante).
// El consumidor (loop()) usa spin_lock_unsafe_blocking (seguro fuera de ISR).
struct TelemetryRingBuffer {
  static constexpr uint32_t CAPACITY = 1000;

  TelemetryPoint buffer[CAPACITY];
  volatile uint32_t head = 0;  // próxima posición de escritura (productor)
  volatile uint32_t tail = 0;  // próxima posición de lectura (consumidor)

  // Retorna la cantidad de elementos almacenados (llamar con lock tomado o aceptar
  // lectura aproximada si se llama sin lock para diagnóstico).
  uint32_t count() const {
    uint32_t h = head;
    uint32_t t = tail;
    return (h >= t) ? (h - t) : (CAPACITY - t + h);
  }

  bool is_full() const {
    return ((head + 1) % CAPACITY) == tail;
  }

  bool is_empty() const {
    return head == tail;
  }

  // Intenta pushear un punto desde la ISR. Usa spin_try_lock_unsafe: si el lock
  // no se consigue, descarta el punto y retorna false (la ISR no debe bloquearse).
  // También descarta si el buffer está lleno.
  bool try_push(const TelemetryPoint& pt, spin_lock_t* lock) {
    if (!spin_try_lock_unsafe(lock)) {
      return false;  // lock no disponible, descartar punto
    }

    uint32_t next_head = (head + 1) % CAPACITY;
    if (next_head == tail) {
      // buffer lleno, descartar
      spin_unlock_unsafe(lock);
      return false;
    }

    buffer[head] = pt;
    head = next_head;
    spin_unlock_unsafe(lock);
    return true;
  }

  // Extrae un punto (llamar desde loop(), no desde ISR). Bloquea hasta obtener
  // el spinlock — aceptable en contexto de thread, no en ISR.
  bool pop(TelemetryPoint& out, spin_lock_t* lock) {
    spin_lock_unsafe_blocking(lock);

    if (head == tail) {
      // buffer vacío
      spin_unlock_unsafe(lock);
      return false;
    }

    out = buffer[tail];
    tail = (tail + 1) % CAPACITY;
    spin_unlock_unsafe(lock);
    return true;
  }
};

//*** SHARED DATA ***********************************************************************

struct SharedData {
  explicit SharedData(int hw_spinlock_id = 0) {
    lock = spin_lock_instance(hw_spinlock_id);
  }

  // posición objetivo actual del carro, en mm (ya interpolada por el perfil trapezoidal)
  volatile float target_position_mm = 0.0f;
  // velocidad del perfil en ese mismo instante [mm/s], para la realimentacion anticipada
  volatile float target_velocity_mm_s = 0.0f;

  // Core 1 solo debe llamar a AxisController::update_servo() mientras esto es true.
  // Se pone en false desde Core 0 antes de correr homing/calibración (que toman
  // control exclusivo y bloqueante del encoder/driver), y en true al terminar.
  // Ver advertencia de coordinación en homing_controller.h del módulo reusado.
  volatile bool servo_loop_enabled = false;

  // --- Telemetría binaria de alta tasa ---
  // Flag controlado por M120 S<1|0>. La ISR lo lee para decidir si empaqueta puntos.
  volatile bool telemetry_enabled = false;
  // M120 P<n>: un punto cada n ticks de la ISR de 2 kHz (1 = 2 kHz, 2 = 1 kHz...)
  volatile uint32_t telemetry_decimation = 1;
  // Ring buffer estático de TelemetryPoint, compartido entre ISR (push) y loop() (pop).
  TelemetryRingBuffer telemetry_buffer;

  // --- Lazo externo de cascada (encoder lineal) ---
  // Flag para habilitar/deshabilitar el corrector de cascada en la ISR.
  // Default false: todavia no hay encoder lineal, y read_linear_encoder() devuelve la
  // misma posicion del motor que ya corrige el lazo servo. Con eso el integrador solo
  // acumula el error de seguimiento y corre el objetivo hasta +2 mm.
  volatile bool cascade_corrector_enabled = false;

  // Reaprendizaje anti-cogging al calibrar (M56): el ISR del perfil acumula el error plegado.
  CoggingLearner cogging_learner;

  spin_lock_t* lock = nullptr;
};
