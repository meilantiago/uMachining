# Plan de Pruebas y Puesta en Marcha (Bring-Up)

Este documento detalla los pasos secuenciales para probar el firmware de control de precisión una vez que tengas acceso al hardware. La clave del éxito en sistemas de control en cascada es **probar los lazos desde adentro hacia afuera**. Nunca cierres el lazo externo (corrector) si el lazo interno (motor) no es estable.

---

## 1. Verificación Inicial y Flasheo

1. **Hardware:** Conecta la Raspberry Pi Pico a la placa base. Asegúrate de que los voltajes lógicos (3.3V) sean correctos.
2. **Motores apagados:** Desconecta temporalmente la alimentación de potencia (VMOT) del driver TB6612 para evitar movimientos bruscos indeseados durante el flasheo.
3. **Flasheo:** Compila el proyecto con CMake y arrastra el archivo `.uf2` a la Pico.
4. **Consola Serial:** Abre tu monitor serial (ej. PuTTY, minicom, o un script de Python) configurado a **921600 baudios**.
5. **Verificación de Boot:** Al iniciar, deberías ver el mensaje de inicialización en la consola (`"Inicializacion terminada. Enviar G28 para homear, M56 para calibrar."`).

---

## 2. Validación del Lazo Interno (Core 1) y Encoder Principal

En esta etapa, el firmware asumirá el control del motor, pero sin correcciones externas. El mock actual de `read_linear_encoder()` ya permite hacer esto porque la corrección será casi cero.

1. **Alimentación:** Conecta la alimentación de potencia al driver TB6612.
2. **Lectura del Encoder:**
   * Gira el motor con la mano (asegúrate de que el comando `M18` se haya enviado para que el motor esté libre).
   * Envía `M119` (Reportar estado). Verifica que el ángulo y la posición reportada cambien de manera coherente.
3. **Calibración:**
   * Envía `M56 P`. Esto ejecutará la rutina de calibración eléctrica del encoder/motor. Presta atención a los valores impresos (`P`) para asegurarte de que el mapeo magnético es limpio.
4. **Homing:**
   * Envía `G28`. El eje debe moverse suavemente hacia el tope mecánico y resetear su posición a `0.00`.
5. **Movimientos en Lazo Cerrado:**
   * Envía `G1 X10.0 F5`. El eje debería moverse 10 milímetros a 5 mm/s.
   * Empuja el carro ligeramente con el dedo; el motor (gobernado por el Core 1) debería resistir y volver a su posición.

---

## 3. Validación de Telemetría (Tarea 1)

Es crítico asegurar que podemos capturar datos sin saturar el procesador.

1. **Script de Captura:** Usa o crea un script básico de Python en tu PC que abra el puerto serie a 921600 baudios, lea los paquetes (Header `0xAA 0x55` + 16 bytes) y los guarde en un CSV.
2. **Activación:** Envía `M120 S1` desde tu script.
3. **Inyección de Movimiento:** Envía un movimiento (ej. `G1 X50 F20`).
4. **Análisis:** Revisa el CSV. Deberías tener datos separados por exactamente ~500 microsegundos (2 kHz). Verifica que `target_mm` siga un perfil trapezoidal perfecto (acelera, velocidad constante, desacelera).

---

## 4. Integración del Encoder Rotativo Secundario (MT6835)

Dado que usarás un segundo **MT6835 rotativo** acoplado a la mecánica (en lugar de un encoder lineal puro), el firmware funcionará de forma idéntica, pero requiere una pequeña adaptación matemática en el código cuando lo vayas a instalar.

1. **Adaptación de Código (Previo a compilar):**
   * Instancia un segundo objeto SPI / MT6835 en `main.cpp` apuntando a otros pines.
   * Modifica la función mock `read_linear_encoder()`:
     ```cpp
     float read_linear_encoder() {
         float angulo_grados = external_encoder->read_angle();
         // Convierte grados a mm lineales (ej. si una vuelta = 2mm de avance)
         return (angulo_grados / 360.0f) * LEAD_SCREW_PITCH_MM; 
     }
     ```
2. **Validación del Sensor Externo:**
   * Con el motor apagado (`M18`), mueve el carro a mano.
   * Revisa en la telemetría que `linear_pos_mm` coincida (con cierta tolerancia por el backlash) con la posición física real. Si se mueve al revés, invierte el signo matemático en la conversión.

---

## 5. Cierre y Sintonización del Lazo en Cascada (Tarea 2)

Una vez que el motor sigue el perfil y el encoder externo mide bien, es hora de habilitar la corrección.

1. **Habilitación:** El flag `cascade_corrector_enabled` está en `true` por defecto. El PI ya está funcionando.
2. **Prueba de Backlash (Rechazo de Perturbaciones):**
   * Haz un movimiento a `X10`.
   * Detenlo y aplica una ligera fuerza externa al carro. El encoder secundario detectará que el carro se movió (aunque el motor siga en su lugar por el juego mecánico).
   * El corrector PI (`LinearCascadeCorrector`) verá el error y aumentará sutilmente el `target_position_mm` del Core 1, obligando al motor a empujar hasta que el encoder externo vuelva a marcar `10.0 mm`.
3. **Ajuste Fino (Tuning):**
   * Utiliza la telemetría para graficar el error (`target_mm - linear_pos_mm`).
   * Si el sistema oscila lentamente, reduce `kp_` (Proporcional) y `ki_` (Integral) en `cascade_corrector.cpp`. Si tarda mucho en corregir la posición final, aumenta `ki_`. 
   * **Importante:** Este lazo es *lento* a propósito. No intentes que responda en milisegundos (para eso está el Core 1); su trabajo es llevar el error estático a cero.
