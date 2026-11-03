// --------------------------------------------------------------------------------------
// Data shared between the two RP2040/RP2350 cores:
//  - Core 0 runs the trapezoidal profile (2 kHz) and writes the current target position.
//  - Core 1 runs the servo loop (as fast as possible) and reads that target.
// Same pattern as 'SharedData' in robot.h of MicroManipulatorStepper (0x23), for a single axis.
//
// Telemetry: a static ring buffer lets the 2 kHz ISR record axis state without blocking;
// loop() drains it as a binary stream.
// --------------------------------------------------------------------------------------

#pragma once

#include <cstdint>
#include "pico/sync.h"
#include "servo_control/cogging_learner.h"

//*** TELEMETRY *************************************************************************

// Packet on the wire: header 0xAA 0x56 + this 28-byte struct, little-endian (parsed by tools/interfaz.py).
// Positions are int32 in POSITION_LSB_MM (0.25 nm) units: a float in mm only resolves ~15 nm at
// 200 mm, which quantizes the tracking error.
struct TelemetryPoint {
  uint32_t timestamp_us;   // time_us_64() truncated to 32 bits (wraps every ~71 min)
  int32_t  target_q;       // target position (trapezoidal profile output) [0.25 nm]
  int32_t  motor_pos_q;    // motor position (cached last encoder reading) [0.25 nm]
  float    vel;            // motor velocity [rad/s]
  float    out;            // velocity loop output [electrical rad], limited to +-0.45*pi
  int32_t  chip_raw;       // raw MT6835 angle (0..2^21-1)
  uint8_t  status;         // encoder status bits (overspeed, weak field, undervoltage)
  uint8_t  flags;          // bit0: motor_update (loop active), bit1: servo fault trip
  uint16_t seq;            // sample counter (gaps = dropped samples)
};
static_assert(sizeof(TelemetryPoint) == 28, "el script de PC espera 28 bytes");

// Static circular buffer, no dynamic allocation.
// Producer (ISR) uses spin_try_lock_unsafe (non-blocking);
// consumer (loop()) uses spin_lock_unsafe_blocking (safe outside ISR context).
struct TelemetryRingBuffer {
  static constexpr uint32_t CAPACITY = 1000;

  TelemetryPoint buffer[CAPACITY];
  volatile uint32_t head = 0;  // next write index (producer)
  volatile uint32_t tail = 0;  // next read index (consumer)

  // Number of stored elements. Exact only with the lock held; approximate otherwise.
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

  // Push from the ISR. If the lock is taken or the buffer is full, the point is dropped
  // and false is returned (the ISR must never block).
  bool try_push(const TelemetryPoint& pt, spin_lock_t* lock) {
    if (!spin_try_lock_unsafe(lock)) {
      return false;
    }

    uint32_t next_head = (head + 1) % CAPACITY;
    if (next_head == tail) {
      spin_unlock_unsafe(lock);
      return false;
    }

    buffer[head] = pt;
    head = next_head;
    spin_unlock_unsafe(lock);
    return true;
  }

  // Pop from loop() only, never from an ISR: blocks until the spinlock is acquired.
  bool pop(TelemetryPoint& out, spin_lock_t* lock) {
    spin_lock_unsafe_blocking(lock);

    if (head == tail) {
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

  // Current carriage target [mm], already interpolated by the profile.
  // double is not written atomically (64 bits), so always access it with 'lock' held.
  volatile double target_position_mm = 0.0;
  // For Core 0 code outside the ISR (the ISR uses spin_try_lock_unsafe and skips the tick
  // if the lock is taken).
  void set_target_position_mm(double mm) {
    uint32_t irq = spin_lock_blocking(lock);
    target_position_mm = mm;
    spin_unlock(lock, irq);
  }
  // profile velocity at the same instant [mm/s], used for velocity feed-forward
  volatile float target_velocity_mm_s = 0.0f;

  // Core 1 may call AxisController::update_servo() only while this is true. Core 0 clears it
  // before homing/calibration (which take blocking, exclusive control of encoder and driver)
  // and sets it again afterwards. See the coordination note in homing_controller.h.
  volatile bool servo_loop_enabled = false;

  // --- High-rate binary telemetry ---
  // Set by M120 S<1|0>; the ISR only records points while it is true.
  volatile bool telemetry_enabled = false;
  // M120 P<n>: one point every n ISR ticks (1 = 2 kHz, 2 = 1 kHz, ...)
  volatile uint32_t telemetry_decimation = 1;
  TelemetryRingBuffer telemetry_buffer;

  // --- Outer cascade loop (linear encoder) ---
  // Off by default: without a linear encoder, read_linear_encoder() returns the motor position
  // the servo loop already controls, so the integrator would only accumulate following error
  // and shift the target by up to +2 mm.
  volatile bool cascade_corrector_enabled = false;

  // Anti-cogging learning during calibration (M56): the profile ISR accumulates the folded error.
  CoggingLearner cogging_learner;

  spin_lock_t* lock = nullptr;
};
