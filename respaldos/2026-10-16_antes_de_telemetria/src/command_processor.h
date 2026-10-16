// --------------------------------------------------------------------------------------
// SingleAxisCommandProcessor: implementa ICommandProcessor (interfaz reusada de
// command_parser.h) con un set de comandos G/M-code reducido a lo que necesita
// 1 solo eje lineal. Reemplaza el switch grande de Robot::process_command() del
// proyecto original, que soportaba 3 ejes + cinemática delta + herramientas PWM.
//
// Comandos soportados:
//   G0 / G1 X<mm> [F<mm/s>]   mover a la posición X (según modo G90/G91)
//   G28                        homing contra el tope mecánico del husillo
//   G90 / G91                  posicionamiento absoluto / relativo
//   M17 / M18                  habilitar / deshabilitar corriente del motor
//   M55 A B C D F               tunear PID en caliente (pos_kp, pos_ki, vel_kp, vel_ki, vel_lowpass_tc)
//   M56 [S] [P] [D<1|-1>]       calibrar (S=guardar en flash, P=imprimir mediciones, D=sentido de homing)
//   M114                        reportar posición actual
//   M119                        reportar estado (homed, calibrado, ángulo de encoder, etc.)
//   M120 S<1|0>                  activar/desactivar streaming de telemetría binaria
// --------------------------------------------------------------------------------------

#pragma once

#include "command_parser/command_parser.h"
#include "axis_controller.h"
#include "motion_profile/trapezoidal_profile.h"
#include "shared_data.h"

//*** CLASS *****************************************************************************

class LinearCascadeCorrector;  // forward-declare (definida en servo_control/cascade_corrector.h)

class SingleAxisCommandProcessor : public ICommandProcessor {
  public:
    SingleAxisCommandProcessor(AxisController* axis, TrapezoidalProfile* profile,
                               SharedData* shared_data, LinearCascadeCorrector* corrector);

    void send_reply(const char* str) override;
    bool can_process_command(const GCodeCommand& cmd) override;
    void process_command(const GCodeCommand& cmd, std::string& reply) override;

  private:
    void process_move_command(const GCodeCommand& cmd, std::string& reply);
    void process_home_command(const GCodeCommand& cmd, std::string& reply);
    void process_tune_pid_command(const GCodeCommand& cmd, std::string& reply);
    void process_calibrate_command(const GCodeCommand& cmd, std::string& reply);
    void process_report_position_command(std::string& reply);
    void process_report_status_command(std::string& reply);
    void process_telemetry_command(const GCodeCommand& cmd, std::string& reply);

  private:
    AxisController* axis;
    TrapezoidalProfile* profile;
    SharedData* shared_data;
    LinearCascadeCorrector* cascade_corrector;

    bool absolute_mode = true;   // G90 (default) vs G91
    float current_target_mm = 0.0f;
    float max_velocity_mm_s = 0.0f;
    float max_accel_mm_s2 = 0.0f;
};
