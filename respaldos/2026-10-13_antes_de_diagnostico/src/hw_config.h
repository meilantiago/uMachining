// --------------------------------------------------------------------------------------
// Proyecto: [nombre de tu proyecto] - eje X de micromaquinado
// Adaptado de MicroManipulatorStepper (0x23), firmware/MotionControllerRP
// Basado en la variante SINGLE_AXIS_BOARD ya prevista en hw_config.h del proyecto original.
// --------------------------------------------------------------------------------------

#pragma once
#include "utilities/math_constants.h"

//--- HUSILLO / MECÁNICA -------------------------------------------------------
// AJUSTAR: paso del husillo de tu eje (avance lineal por vuelta completa del motor)
// Medido el 02/10/2026: el husillo avanza 5 mm por vuelta (antes estaba en 2 mm, por eso
// G1 X10 movia ~25 mm).
constexpr float LEAD_SCREW_PITCH_MM = 5.0f; // [mm/vuelta]

// Límites blandos de recorrido, usados para rechazar comandos G0/G1 fuera de rango.
// Recorrido util medido: ~260 mm. Se dejan 5 mm de margen contra el tope opuesto.
constexpr float AXIS_MIN_POSITION_MM = 0.0f;
constexpr float AXIS_MAX_POSITION_MM = 255.0f;

//--- PERFIL DE MOVIMIENTO (trapezoidal) ---------------------------------------
// AJUSTAR: velocidad y aceleración máximas del carro en mm/s y mm/s^2.
// Empezar conservador (bajo) y subir una vez que el lazo cerrado esté estable.
// El lazo de posicion limita la velocidad que le pide al motor a 2*pi rad/s (1 vuelta/s =
// 5 mm/s con el husillo de 5 mm, ver servo_controller.cpp). El perfil tiene que ir mas
// lento para que el servo tenga margen de recuperar atraso; si no, el error crece en los
// movimientos largos hasta disparar el corte por falla. 2 mm/s = 0.4 vueltas/s (margen
// x2.5; G1 X200 llego bien a 0.5 vueltas/s).
constexpr float DEFAULT_MAX_VELOCITY_MM_S = 2.0f;
constexpr float DEFAULT_MAX_ACCEL_MM_S2   = 5.0f;

//--- MOTOR ---------------------------------------------------------------------
// pole pairs del MOTOR paso a paso (no tiene nada que ver con los imanes de la rueda):
//  * 100 para motores de 0.9°/paso
//  * 50  para motores de 1.8°/paso (200 pasos/vuelta) <- el nuestro
// Verificado con los barridos de encoder: con 50 pares de polos dan 30 periodos de
// encoder por vuelta, que es lo que corresponde a la rueda de 60 imanes.
constexpr float MOTOR_POLE_PAIRS = 50;

// factor de corriente máxima en [0..1]. Valores por debajo de 0.4 reducen mucho
// la resolución del PWM, no se recomienda bajar de ahí.
constexpr float MOTOR_MAX_CURRENT_FACTOR = 0.6f;

//--- ENCODER ROTATIVO (MT6835, montado en el eje del motor) -------------------
// Rueda de imanes alternados N-S: el angulo del encoder da una vuelta (un periodo de 2*PI)
// cada DOS imanes. Con 60 imanes, una vuelta del motor son exactamente 30 periodos.
// (Antes se calculaba con paso 3 mm y radio 28.9 mm, que daba 30.26 periodos: aproximado.)
// AJUSTAR si se cambia la rueda.
constexpr int ENCODER_MAGNET_COUNT = 60;
constexpr int ENCODER_PERIODS_PER_REV = ENCODER_MAGNET_COUNT / 2;
// Conversion de angulo de encoder a angulo de rotor. Se usa antes de tener calibracion
// (homing y tabla por defecto); despues de M56 manda la tabla medida.
constexpr float ENCODER_ANGLE_TO_ROTOR_ANGLE = 1.0f / ENCODER_PERIODS_PER_REV;

// habilita el chequeo de CRC del encoder (algo más lento) - útil para debug.
// Nota (heredada del proyecto original): algunos chips devuelven siempre CRC=0,
// lo que genera falsos errores masivos. Probar con false primero.
constexpr bool ENABLE_ENCODER_CRC = false;

//--- HOMING (contra el tope mecánico del husillo) ------------------------------
// [rad/s de motor] velocidad de busqueda del tope. El homing es en lazo abierto: si el
// rotor pierde un paso, el encoder avanza menos de lo esperado y eso se confunde con el
// tope. Mas lento = mas torque disponible y menos patinazos. 2 rad/s = 1.6 mm/s con el
// husillo de 5 mm: recorrer los 260 mm tarda ~2.7 min en el peor caso.
// (Antes 3 rad/s, que con el husillo de 5 mm eran 2.4 mm/s.)
constexpr float HOMING_VELOCITY = 2.0f;
// El homing busca el tope a lo largo de todo el recorrido mas un margen, asi funciona
// arranque donde arranque el carro.
constexpr float HOMING_SEARCH_RANGE_MM = AXIS_MAX_POSITION_MM + 15.0f;
// [0..1] corriente durante el homing. Antes era 0.4 (reducida): con el husillo de 5 mm
// el motor necesita mas torque para mover el carro, y a 0.4 el rotor patinaba en lazo
// abierto y el homing frenaba en cualquier lado. Igual a la corriente del lazo cerrado.
constexpr float HOMING_CURRENT  = MOTOR_MAX_CURRENT_FACTOR;

//--- CALIBRACIÓN (LUT encoder<->motor, LUT motor<->campo) ---------------------
// Debe cubrir al menos una vuelta completa del motor: la tabla se repite cada vuelta
// para cubrir todo el recorrido del husillo.
constexpr float CALIBRATION_RANGE = 370.0f;         // [grados] desde la posición de home
constexpr float CALIBRATION_FIELD_VELOCITY = 20.0f; // velocidad del campo durante calibración
constexpr int   ENCODER_LUT_SIZE = 1024;             // tamaño de la lookup table (~0.36 grados por celda)

//--- VERIFICACION DE CONMUTACION ----------------------------------------------
// El homing fija el cero del encoder modulo UN periodo del encoder (1/30 de vuelta = 12° de
// eje), pero la conmutacion necesita la posicion modulo un ciclo electrico (7.2° de eje).
// Si el home cae del otro lado del borde del periodo (p.ej. 359° en vez de 1°), la posicion
// se corre 12° de eje = 600° electricos = 240°: el campo empuja para el lado contrario y el
// lazo se clava con la salida saturada. Por eso despues de cada G28 (y al final de M56) se
// gira el campo en lazo abierto, ida y vuelta, y se mide el desfase real entre el campo y lo
// que dice la tabla. Un periodo de mas o de menos da 120° o 240° de desfase: se corrige.
constexpr float COMMUTATION_CHECK_CYCLES = 2.0f;     // ciclos electricos por pasada (2 = 14.4° de eje = 0.2 mm)
constexpr float COMMUTATION_MAX_OFFSET_DEG = 45.0f;  // desfase residual maximo aceptado [grados electricos]

//--- LAZO DE CONTROL (PID en cascada posición -> velocidad) -------------------
// AJUSTAR: estos valores vienen del proyecto original (brazos delta livianos).
// Con el carro + husillo la inercia y fricción son distintas: usar M55 para
// retunear en caliente sin recompilar antes de fijar estos defaults.
constexpr float POS_KP = 60.0f;
constexpr float POS_KI = 30000.0f;

constexpr float VEL_LOWPASS_TC = 0.004f;
constexpr float VEL_KP = 0.2f;
constexpr float VEL_KI = 90.0f;

//--- PINES (variante de placa de 1 solo eje) -----------------------------------
// Mismo pinout que la opción "SINGLE_AXIS_BOARD" del hw_config.h original.
// AJUSTAR si tu propia placa usa otro mapeo de pines.
#define PIN_BUILTIN_LED   25
#define PIN_USER_BUTTON   24

#define PIN_M1_PWM_A_POS  2
#define PIN_M1_PWM_A_NEG  3
#define PIN_M1_PWM_B_POS  5
#define PIN_M1_PWM_B_NEG  6

#define PIN_MOTOR_EN      8
#define PIN_MOTOR_PWM_A   4
#define PIN_MOTOR_PWM_B   7

#define PIN_ENCODER1_CS   17
#define PIN_ENCODER_SCK   18
#define PIN_ENCODER_MISO  20
#define PIN_ENCODER_MOSI  19
