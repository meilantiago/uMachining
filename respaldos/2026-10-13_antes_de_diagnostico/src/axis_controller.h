// --------------------------------------------------------------------------------------
// AxisController: equivalente simplificado de 'RobotJoint' del proyecto original,
// pero para un único eje lineal (carro + husillo) en vez de una junta rotativa de
// un brazo delta. Se encarga de:
//   - inicializar encoder MT6835 + driver TB6612
//   - convertir entre posición lineal del carro (mm) y posición angular del motor (rad),
//     usando el paso del husillo (LEAD_SCREW_PITCH_MM)
//   - homing contra el tope mecánico (reusa HomingController tal cual)
//   - calibración de las lookup tables encoder<->motor y motor<->campo (reusa
//     measure_calibration_data() tal cual)
//   - persistencia de esas LUT en flash (LittleFS)
// --------------------------------------------------------------------------------------

#pragma once

#include <string>
#include "hardware/MT6835_encoder.h"
#include "hardware/TB6612_motor_driver.h"
#include "servo_control/servo_controller.h"

//*** CLASS *****************************************************************************

class AxisController {
  public:
    AxisController(MT6835Encoder* encoder, TB6612MotorDriver* motor_driver,
                   float motor_pole_pairs, float lead_screw_pitch_mm);
    ~AxisController();

    // Inicializa el hardware (encoder + driver) y carga calibración guardada, si existe.
    void init();

    // Corre el homing contra el tope mecánico del husillo. Direction_sign define
    // el sentido de búsqueda (+1 o -1). BLOQUEANTE: no debe correr en paralelo
    // con update_servo() -- coordinar deshabilitando el update en Core 1 antes de
    // llamar a esto (ver comentario en homing_controller.h del módulo reusado).
    // search_stop = false: toma como home el lugar donde esta el rotor, sin buscar tope
    // (para probar con el motor desacoplado).
    // Si hay calibracion, al final verifica la conmutacion y corrige el periodo del
    // encoder (ver verify_commutation); si no coincide, el homing falla.
    bool home(float direction_sign, bool search_stop = true);

    // Corre la secuencia de calibración completa (homing + medición de LUTs).
    // store_to_flash: si true, graba las LUT resultantes en LittleFS para no
    // tener que recalibrar en cada reset.
    bool calibrate(float homing_direction_sign, bool store_to_flash, bool print_measurements,
                   bool search_stop = true);

    // Actualiza el lazo servo (llamar tan seguido como se pueda, idealmente Core 1
    // dedicado). target_position_mm es la posición objetivo actual del carro.
    void update_servo(float target_position_mm, float dt, float one_over_dt);

    // Habilita/deshabilita la corriente del motor.
    void set_motor_enabled(bool enable);

    // Posición actual del carro en mm, según la última lectura de encoder usada
    // en update_servo() (no dispara una lectura nueva).
    float get_position_mm() const;

    // Error de posición de la última actualización del lazo, en mm.
    float get_position_error_mm() const;

    bool is_homed() const;
    bool is_calibrated() const;

    // true si el lazo se corto solo por error de seguimiento sostenido (ver update_servo)
    bool has_servo_fault() const { return servo_fault; }

    // Estado del lazo en el instante del ultimo corte por falla (lo muestra M119)
    struct FaultInfo {
      uint32_t count = 0;   // cortes desde el arranque
      float target_mm = 0.0f, pos_mm = 0.0f, vel = 0.0f, out = 0.0f;
      uint8_t status = 0;   // bits de estado del encoder
    };
    const FaultInfo& get_last_fault() const { return last_fault; }

    // Imprime por serie (CSV) la traza del lazo cerrado: una muestra por ms de los
    // ultimos ~2 s con el lazo activo. Se congela cuando salta el corte por falla.
    void dump_trace();

    // Acceso directo al ServoController interno, útil para tunear PID en caliente (M55).
    ServoController* get_servo_controller();

    // Acceso al encoder, útil para telemetría (M119: ángulo crudo, errores de CRC, etc.)
    MT6835Encoder* get_encoder();

  private:
    // Mide en lazo abierto el desfase entre el campo y la tabla de calibracion (motor
    // energizado, lazo servo detenido). allow_period_correction: si el desfase indica que el
    // contador del encoder quedo corrido un periodo (120°/240°), lo corrige; si no, solo
    // acepta el contador como esta. Devuelve false si el desfase no es aceptable.
    bool verify_commutation(float away_from_stop_sign, bool allow_period_correction);

    float mm_to_motor_pos_rad(float pos_mm) const;
    float motor_pos_rad_to_mm(float motor_pos_rad) const;

    std::string enc_to_pos_lut_filename() const;
    std::string pos_to_field_lut_filename() const;

    bool load_calibration();
    bool store_calibration();

  private:
    MT6835Encoder* encoder;
    TB6612MotorDriver* motor_driver;
    ServoController* servo_controller;

    float lead_screw_pitch_mm;
    float last_position_mm = 0.0f;

    bool homed = false;
    bool calibrated = false;
    volatile bool servo_fault = false;
    uint64_t fault_since_us = 0;
    FaultInfo last_fault;

    struct TracePoint {
      uint32_t t_us;
      float target_mm, pos_mm, vel, out;
      int32_t chip_raw;
      uint16_t loops;   // iteraciones del lazo desde la muestra anterior
      uint8_t status;
    };
    static constexpr int TRACE_SIZE = 2000;
    TracePoint trace[TRACE_SIZE];
    volatile int trace_head = 0;
    volatile int trace_count = 0;
    volatile bool trace_frozen = false;
    uint64_t last_trace_us = 0;
    uint16_t loops_since_trace = 0;
};
