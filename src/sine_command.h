// --------------------------------------------------------------------------------------
// M60: senoide en la posición comandada, para medir la respuesta en frecuencia.
//
//   M60 A<µm> F<Hz> N<ciclos> [R<s>] [T<s>]   arranca (R: entrada/salida, T: asentamiento)
//   M60 S0                                    frena suave
//   M60                                       informa el estado
//
// La senoide se suma al perfil trapezoidal: se puede mandar con el carro quieto o
// durante un G1 (senoide sobre rampa). Mientras dura la ventana de medición el pin
// PIN_SINE_SYNC queda en alto y la telemetría marca flags bit2.
//
// La interpretación y validación están acá, sin dependencias del hardware, para que el
// procesador de comandos del firmware y los tests de PC usen el mismo código.
// --------------------------------------------------------------------------------------

#pragma once

#include <string>
#include "command_parser/command_parser.h"
#include "motion_profile/sine_profile.h"

// Estado del eje en el momento del comando
struct SineContext {
  bool ready = false;          // homeado y calibrado
  bool motor_enabled = false;  // lazo activo
  double low_mm = 0.0;         // menor posición que va a recorrer el perfil trapezoidal
  double high_mm = 0.0;        // mayor posición que va a recorrer el perfil trapezoidal
  double trap_speed_mm_s = 0.0;  // velocidad máxima del movimiento trapezoidal en curso (0 si quieto)
  bool sine_active = false;    // ya hay una senoide en curso
};

struct SineAction {
  enum class Kind { START, STOP, STATUS, ERROR };
  Kind kind = Kind::ERROR;
  SineParams params;
  std::string message;   // texto de error (Kind::ERROR)
};

// Interpreta y valida un M60. No toca el generador: START trae los parámetros listos.
SineAction parse_sine_command(const GCodeCommand& cmd, const SineContext& ctx);

// Respuestas para el puerto serie (terminan en "ok\n")
std::string sine_start_reply(const SineProfile& sine);
std::string sine_status_reply(const SineProfile& sine);

// Ejecuta un M60 completo y devuelve la respuesta. 'lock'/'unlock' protegen el generador de
// la interrupción de 2 kHz mientras se lo modifica (en el firmware deshabilitan las IRQ; en
// los tests pueden ser nullptr). ctx.sine_active se completa acá.
std::string execute_sine_command(const GCodeCommand& cmd, SineContext ctx, SineProfile& sine,
                                 uint32_t (*lock)(), void (*unlock)(uint32_t));
