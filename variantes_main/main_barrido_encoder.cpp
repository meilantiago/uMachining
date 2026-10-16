// --------------------------------------------------------------------------------------
// main.cpp TEMPORAL - Caracterizacion del encoder MT6835 frente a la rueda de imanes.
// (El firmware real esta respaldado en main_firmware_completo.cpp.bak, en la raiz.)
//
// Al recibir "G56" por serie: gira el motor en lazo abierto 2 vueltas en un sentido y
// 2 vueltas en el sentido contrario, y manda por serie una muestra cada 1 ms con:
//   D,<t_us>,<sentido>,<angulo de campo comandado [rad]>,<angulo crudo del chip>,<status>
// status: bit0 overspeed, bit1 campo debil, bit2 subtension, bit3 error de CRC.
// El MT6835 no informa la intensidad del campo: solo el bit de campo debil.
// Usar tools/encoder_sweep.py para capturar, guardar el CSV y graficar.
// --------------------------------------------------------------------------------------

#include <Arduino.h>

#include "hw_config.h"
#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "utilities/math_constants.h"

MT6835Encoder* encoder = nullptr;
TB6612MotorDriver* motor_driver = nullptr;

constexpr float SWEEP_REVOLUTIONS = 2.0f;       // vueltas mecanicas por sentido
constexpr float SWEEP_SPEED_REV_S = 0.25f;      // velocidad de crucero [vueltas/s de motor]
constexpr float SWEEP_RAMP_S = 0.5f;            // rampa de aceleracion/frenado [s]
constexpr float MOTOR_AMPLITUDE = MOTOR_MAX_CURRENT_FACTOR;
constexpr uint32_t FIELD_UPDATE_US = 100;       // actualizacion del campo
constexpr uint32_t SAMPLE_EVERY = 10;           // 1 muestra cada 10 actualizaciones = 1 ms

String serial_line;

// Mueve el campo 'revolutions' vueltas mecanicas en el sentido 'dir' (+1/-1) con perfil
// trapezoidal de velocidad, enviando una muestra del encoder cada 1 ms.
void sweep_leg(int dir, uint64_t t0_us) {
  const float total = SWEEP_REVOLUTIONS * Constants::TWO_PI_F * MOTOR_POLE_PAIRS; // rad electricos
  const float v_max = SWEEP_SPEED_REV_S * Constants::TWO_PI_F * MOTOR_POLE_PAIRS; // rad/s electricos
  const float accel = v_max / SWEEP_RAMP_S;
  const float dt = FIELD_UPDATE_US * 1e-6f;

  float field = motor_driver->get_field_angle();
  float traveled = 0.0f, v = 0.0f;
  uint32_t n = 0;
  uint64_t next_us = time_us_64();

  while (traveled < total) {
    // frenar a tiempo para terminar en 'total'
    float remaining = total - traveled;
    if (v * v / (2.0f * accel) >= remaining) v = fmaxf(v - accel * dt, 0.05f * v_max);
    else if (v < v_max) v = fminf(v + accel * dt, v_max);

    float step = fminf(v * dt, remaining);
    traveled += step;
    field += dir * step;
    motor_driver->set_field_angle(field);

    if (n++ % SAMPLE_EVERY == 0) {
      int32_t raw = encoder->read_abs_angle_raw();
      (void)raw;
      int32_t chip_raw = encoder->get_last_chip_raw_angle();
      uint8_t status = encoder->get_status();
      Serial.printf("D,%lu,%d,%.5f,%ld,%u\n", (unsigned long)(time_us_64() - t0_us), dir, field,
                    (long)chip_raw, (unsigned)status);
    }

    next_us += FIELD_UPDATE_US;
    while (time_us_64() < next_us) {}
  }
}

void run_sweep() {
  Serial.printf("META,pole_pairs=%d,cpr=%d,revolutions=%.2f,speed_rev_s=%.3f,amplitude=%.2f,bandwidth=5,hysteresis=4\n",
                (int)MOTOR_POLE_PAIRS, MT6835_CPR, SWEEP_REVOLUTIONS, SWEEP_SPEED_REV_S, MOTOR_AMPLITUDE);
  Serial.println("BEGIN");

  motor_driver->set_amplitude_smooth(MOTOR_AMPLITUDE, 200);
  delay(300); // que el rotor se asiente en el angulo inicial
  encoder->get_crc_error_count(true);

  uint64_t t0 = time_us_64();
  sweep_leg(+1, t0);
  delay(200);
  sweep_leg(-1, t0);

  Serial.printf("END,crc_errors=%lu\n", (unsigned long)encoder->get_crc_error_count(false));
  motor_driver->set_amplitude_smooth(0.0f, 200);
}

void setup() {
  Serial.begin(921600);
  while (!Serial) {}
  delay(100);

  MT6835Encoder::setup_spi(spi0, PIN_ENCODER_SCK, PIN_ENCODER_MOSI, PIN_ENCODER_MISO, 8000000);
  encoder = new MT6835Encoder(spi0, PIN_ENCODER1_CS);
  if (!encoder->init(0x5, 0x4))
    Serial.println("ERROR: no se pudo inicializar el encoder");
  encoder->set_crc_enabled(true);

  motor_driver = new TB6612MotorDriver(
    PIN_MOTOR_EN, PIN_M1_PWM_A_POS, PIN_M1_PWM_A_NEG, PIN_MOTOR_PWM_A,
    PIN_MOTOR_EN, PIN_M1_PWM_B_POS, PIN_M1_PWM_B_NEG, PIN_MOTOR_PWM_B
  );
  motor_driver->begin();
  motor_driver->enable();
  motor_driver->set_amplitude(0.0f, true);

  Serial.println("Listo. Enviar 'G56' para el barrido (2 vueltas ida + 2 vuelta).");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      serial_line.trim();
      if (serial_line == "G56")
        run_sweep();
      serial_line = "";
    } else {
      serial_line += c;
    }
  }
}
