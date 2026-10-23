// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#pragma once

#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "utilities/lookup_table.h"
#include "pid.h"

//*** CLASS *****************************************************************************

class ServoController {
  public:
    // use defines instead of virtual functions for speed
    // TODO: check if this makes any difference and change accordingly
    typedef TB6612MotorDriver MOTOR_DRIVER_TYPE;
    typedef MT6835Encoder ENCODER_TYPE;

  public:
    ServoController(MOTOR_DRIVER_TYPE& motor_driver, ENCODER_TYPE& encoder, int32_t motor_pole_pairs);

    // initialize the servo controller hardware
    void init(float max_motor_amplitude);

    // set the encoder raw angle to motor position lookup table
    void set_enc_to_pos_lut(LookupTable& lut);
    // get the encoder raw angle to motor position lookup table
    const LookupTable& get_enc_to_pos_lut() const;

    // set the motor position to field angle lookup table
    void set_pos_to_field_lut(LookupTable& lut);
    // get the motor position to field angle lookup table
    const LookupTable& get_pos_to_field_lut() const;

    // Updates the servo loop. velocity_ff: velocidad del perfil [rad/s] que se suma a la que
    // pide el lazo de posicion (realimentacion anticipada; solo si feedforward_enabled).
    // Posiciones en double: en float, a 255 mm (320 rad) van de a 3e-5 rad = 24 nm.
    void update(double target_motor_pos,
                float dt,
                float one_over_dt,
                float velocity_ff = 0.0f);

    // M58: como se estima la velocidad y si se usa la realimentacion anticipada (ver hw_config.h)
    volatile bool velocity_from_lut = true;
    volatile bool feedforward_enabled = true;

    // Compensacion anticipada de la retencion del motor (anti-cogging, M59). Tabla de COGGING_TABLE_SIZE
    // puntos sobre un ciclo electrico, indexada por el angulo electrico del campo (absoluto: no depende
    // del homing ni de la calibracion). Se suma como adelanto del campo [rad electricos] a la salida del
    // PI. 'output' (y la telemetria) sigue siendo solo la salida del PI.
    static constexpr int COGGING_TABLE_SIZE = 64;
    float cogging_table[COGGING_TABLE_SIZE] = {};       // parte igual en los dos sentidos [rad el.]
    // Parte que cambia de signo con el sentido de marcha: el lazo responde distinto a un mismo patron segun
    // el sentido (ida y vuelta ven el patron con fases conjugadas). Se multiplica por la velocidad del
    // perfil / cogging_dir_ref, limitado a +-1 (0 quieto, suave al arrancar y al frenar).
    float cogging_table_dir[COGGING_TABLE_SIZE] = {};
    volatile float cogging_dir_ref = 1.0f;              // [rad/s de motor] velocidad a la que se aprendio
    volatile bool cogging_enabled = false;
    volatile float cogging_gain = 1.0f;
    float cogging_feedforward(float field_angle, float velocity_ff) const;

    // Checks if the motor is at position (uses values from previous update() call).
    bool at_position(float motor_pos_eps);

    // Reads the current motor position from the encoder.
    double read_position();

    // Returns the motor position.
    double get_position();

    // Returns the current position error from the last servo loop update.
    float get_position_error();

    // Velocity estimate and loop output (field angle offset) from the last update.
    float get_velocity() const { return velocity; }
    float get_output() const { return output; }
    bool is_motor_update_enabled() const { return motor_update_enabled; }

    // Moves to a new motor position using closed loop control (blocking).
    bool move_to(float target_motor_angle, 
                 float at_pos_motor_angle_eps, 
                 float settle_time_ms,
                 float timeout_us);

    // Moves to a new motor position using open loop controll (blocking).
    // Motor updates must be disabled if servo loop is running in background.
    void move_to_open_loop(float target_motor_angle, 
                           float angular_velocity);
    
    // Returns the encoder object.
    ENCODER_TYPE& get_encoder();

    // Returns the motor driver object.
    MOTOR_DRIVER_TYPE& get_motor_driver();

    // Returns number of motor pole pairs.
    // Can be used to approximate conversion of motor position to field angle.
    float get_pole_pair_count();

    // enable or disable motor 
    void set_motor_enabled(bool enable, bool synchronize_field_angle);

    // enable or disable motor updates
    void set_motor_update_enabled(bool enable);

    // enable or disable encoder reads
    void set_encoder_update_enabled(bool enable);

    // Relee el encoder y reinicia posicion previa, filtro de velocidad y PID. Llamar con el
    // lazo servo detenido, despues de mover el motor por fuera de update() (homing,
    // calibracion), para que el primer update() no vea un salto enorme de cuentas.
    void resync();

  public:
    double encoder_angle_to_motor_pos(ENCODER_TYPE::AbsRawAngleType encoder_angle_raw);
    float motor_pos_to_field_angle(float motor_pos);
    float motor_pos_to_field_angle_derivative(float motor_pos);

  public:
    LowpassFilter velocity_lowpass;
    PIDController pos_controller;
    PIDController velocity_controller;

  private:
    ENCODER_TYPE& encoder;
    MOTOR_DRIVER_TYPE& motor_driver;
    LookupTable encoder_raw_to_motor_pos_lut;
    LookupTable motor_pos_to_field_angle_lut;

    float motor_pole_pair_count = 0.0f;   // number as motor pole pairs (as float to avoid repeated conversion)
    float motor_current_amplitude = 0.5f; // motor current in range [0..1]
    double motor_pos = 0;                 // current motor position
    double motor_pos_prev = 0;            // previous motor position
    // vueltas enteras de la ultima lectura y su desplazamiento en rad: cambia una vez por vuelta,
    // asi el lazo no hace la multiplicacion en double en cada pasada
    int32_t lut_periods = INT32_MIN;
    double lut_periods_offset = 0.0;
    ENCODER_TYPE::AbsRawAngleType encoder_angle_raw_prev = 0; // previous encoder reading
    float pos_error = 0;                  // current position error as computed by upate() (diferencia de doubles)
    float velocity = 0;                   // current velocity estimate
    float output = 0.0f;                  // servo loop output (field angle offset)
    volatile bool motor_update_enabled = false; // enables mootor field updates (Core 0 la escribe, Core 1 la lee)
    bool encoder_update_enabled = true;   // enables encoder reads
};