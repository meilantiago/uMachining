// --------------------------------------------------------------------------------------
// SingleAxisCommandProcessor: ICommandProcessor implementation with a G/M-code set reduced
// to a single linear axis. Replaces Robot::process_command() from the original
// MicroManipulatorStepper project by 0x23 (3 axes, delta kinematics, PWM tools).
//
// Supported commands:
//   G0 / G1 X<mm> [F<mm/s>]     move to X (absolute or relative, see G90/G91); F is capped
//                               by the M203 limit
//   G28 [H]                     home against the negative mechanical stop of the screw;
//                               H = take the current position as home without searching the stop
//   G90 / G91                   absolute (default) / relative positioning
//   M17 / M18                   enable / disable motor current (M17 requires M56 and G28 first
//                               and holds the carriage where it is; resend G1 afterwards)
//   M55 A B C D F               set PID parameters at runtime: A=pos_kp B=pos_ki C=vel_kp
//                               D=vel_ki F=vel_lowpass_tc (all five required)
//   M56 [S] [P] [H] [F<0|1>] [A<0|1>]
//                               calibrate: S=store to flash, P=print measurements, H=calibrate
//                               at the current position without searching the stop,
//                               F=filter motor harmonics out of the table,
//                               A=relearn the anti-cogging tables afterwards
//   M57 X<mm> [F<mm/s>] [P<n>]  diagnostic open-loop sweep (one direction) printing the
//                               commutation offset every n steps; send M17 afterwards
//   M58 [V<0|1>] [F<0|1>]       V=velocity from the LUT slope (1) or from raw counts (0);
//                               F=profile velocity feedforward on/off
//   M59 [...]                   anti-cogging compensation, no arguments = report status:
//                               S<0|1> enable, G<gain>, C clear tables,
//                               I<idx> V<el. deg> direction-independent entry,
//                               J<idx> V<el. deg> direction-dependent entry (scales with speed),
//                               R<mm/s> reference speed of the direction-dependent part,
//                               P print tables, W store to flash, L load from flash
//   M114                        report current position
//   M119                        report status (homed, calibrated, encoder angle, last fault, ...)
//   M120 S<1|0> [P<n>]          binary telemetry on/off (P: one sample every n 2 kHz ticks)
//   M121                        print the closed-loop trace buffer (CSV)
//   M203 [X<mm/s>]              set / report the maximum G0/G1 velocity
//
// G28 and M56 also accept D-1 (homing towards the negative stop); D1 is rejected.
// --------------------------------------------------------------------------------------

#pragma once

#include "command_parser/command_parser.h"
#include "axis_controller.h"
#include "motion_profile/trapezoidal_profile.h"
#include "shared_data.h"

//*** CLASS *****************************************************************************

class LinearCascadeCorrector;  // defined in servo_control/cascade_corrector.h

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
    double current_target_mm = 0.0;
    float max_velocity_mm_s = 0.0f;
    float max_accel_mm_s2 = 0.0f;
};
