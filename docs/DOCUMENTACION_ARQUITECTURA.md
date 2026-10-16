# Arquitectura y Diseño del Firmware de Control de Precisión (Eje Único)

Este documento detalla la arquitectura, decisiones de diseño y el flujo de procesamiento del firmware desarrollado para el control de bucle cerrado de un eje lineal de alta precisión, implementado sobre la arquitectura RP2040/RP2350 (Raspberry Pi Pico SDK).

El objetivo principal de este sistema es lograr un posicionamiento de precisión sub-milimétrica, mitigando imperfecciones mecánicas (como el *backlash* o derivas térmicas) y permitiendo la captura de datos de alta fidelidad para su posterior validación metrológica (ej. comparación con interferometría láser).

---

## 1. Topología del Sistema y Arquitectura Dual-Core

Para garantizar que los estrictos requisitos de tiempo real del control del motor no se vean afectados por tareas no deterministas (como el procesamiento de strings G-code o el envío de datos por USB/Serial), el firmware adopta una **arquitectura de procesamiento asimétrico dual-core**.

### Core 0: Planificación, Comunicaciones y Lazo Externo
El Core 0 actúa como el cerebro supervisor del sistema. Sus responsabilidades incluyen:
* **Procesamiento de Comandos (G-code):** Recibe, parsea y despacha comandos de forma asíncrona.
* **Perfilado de Movimiento (Trapezoidal Profile):** Genera la trayectoria ideal suave (posición y velocidad) que el eje debe seguir, respetando los límites físicos de aceleración y velocidad. Se ejecuta de manera rigurosa en una rutina de interrupción de temporizador (ISR) a 2 kHz.
* **Gestión de Telemetría:** Extrae los datos recolectados y los transmite al host sin bloquear el control.

### Core 1: Lazo Interno FOC (Field Oriented Control)
El Core 1 está dedicado de forma exclusiva y en un bucle infinito al control directo del hardware del motor:
* **Servo Controlador:** Lee la posición del eje del motor (encoder rotativo magnético MT6835) y aplica las corrientes necesarias a través del driver (TB6612) para que el motor alcance la posición dictada por el Core 0.
* **Aislamiento de Tareas:** Al no tener que lidiar con interrupciones de comunicación ni cálculos de trayectoria de alto nivel, el Core 1 garantiza una respuesta instantánea y determinista a las perturbaciones del motor.

---

## 2. Estrategia de Control y Multiplicación de Resolución Magnética

Para lograr la máxima precisión, el sistema no confía ciegamente en pasos discretos. Implementamos una estrategia de **FOC (Field Oriented Control)** utilizando un único **encoder magnético absoluto (MT6835)**.

Sin embargo, el diseño mecánico cuenta con una innovación crucial para aumentar la precisión: el encoder no está leyendo un simple imán diametral en el eje del motor. En su lugar, el encoder está posicionado frente a una **rueda magnética multipolar** acoplada mecánicamente.

### 2.1. Aumento de Resolución por Múltiples Polos
Al utilizar un anillo de imanes (caracterizado en el firmware por `ENCODER_MAGNET_PITCH` y `ENCODER_MAGNET_RADIUS`), el sensor atraviesa múltiples períodos magnéticos por cada revolución mecánica completa del eje. 
Esto significa que los 21 bits de resolución del MT6835 no se distribuyen en 360 grados mecánicos, sino en una fracción menor, **multiplicando drásticamente la resolución efectiva** del sistema de posicionamiento.

### 2.2. Compensación y Calibración (LUT)
Para que el algoritmo FOC (que asume un mapeo directo de ángulo eléctrico) funcione con esta topología multipolar, el firmware integra una **Tabla de Búsqueda (Lookup Table - LUT)**. Durante la inicialización (homing/calibración), el sistema correlaciona las lecturas del encoder con los ángulos eléctricos reales del motor, neutralizando cualquier no-linealidad en la disposición de los imanes del anillo. El Core 1 utiliza esta tabla en tiempo real a altísima velocidad para aplicar los vectores de corriente PWM correctos.

---

## 3. Sincronización Segura: Hardware Spinlocks

Dado que el Core 0 y el Core 1 operan de forma independiente y asíncrona, necesitan compartir información (el Core 0 escribe el *target*, el Core 1 lo lee).
Si ambos núcleos acceden a la memoria simultáneamente, pueden ocurrir condiciones de carrera (*race conditions*) provocando lecturas corruptas y movimientos erráticos del motor.

**Solución Implementada:**
Se utiliza el `SharedData` protegido por un **Hardware Spinlock** del RP2040. 
Cuando un núcleo necesita leer o escribir datos críticos, toma el "candado" por hardware, realiza la operación en un puñado de ciclos de reloj, y lo suelta.
* **Consideración Crítica en ISR:** Dentro de la ISR del Core 0, no podemos quedarnos esperando eternamente si el Core 1 tiene el candado. Por ello, la ISR usa `spin_try_lock_unsafe()`; si el candado está ocupado, la ISR simplemente omite la actualización en ese ciclo (a 2 kHz, perder un ciclo de 500 microsegundos es imperceptible) previniendo bloqueos fatales (*deadlocks*) en el sistema.

---

## 4. Validación Metrológica: Telemetría Binaria de Alta Tasa

Para el desarrollo del proyecto, no basta con que el sistema funcione; **es imperativo probar empíricamente su precisión** comparando el desempeño del firmware contra instrumentos de laboratorio (interferómetro).
Los típicos mensajes de texto (ej. `Serial.print()`) son demasiado lentos y bloquean el procesador, impidiendo ver qué hace el motor milisegundo a milisegundo.

**Implementación (Tarea 1):**
Hemos diseñado un subsistema de **Telemetría Binaria de Alta Tasa**:
1. **Recolección No Bloqueante:** A 2 kHz, dentro de la misma ISR del perfilador de movimiento, el sistema empaqueta el estado exacto (Timestamp, Target del Perfil, Posición del Motor, Posición Lineal Real) en un *Ring Buffer* (cola circular estática de 1000 elementos).
2. **Desacople en Memoria:** Esta estructura no usa asignación de memoria dinámica (`malloc`/`new`), garantizando que la ISR nunca sufra pausas por recolección de basura o fragmentación, manteniendo el jitter del control cercano a cero.
3. **Transmisión Asíncrona (Drain):** En el bucle principal (`loop()`) del Core 0, el procesador retira los datos del *Ring Buffer* y los transmite masivamente por USB Serial a 921600 baudios usando un protocolo binario altamente empaquetado (18 bytes por muestra).

**Justificación:** Esto nos permite extraer "radiografías" milisegundo a milisegundo del comportamiento del motor bajo cargas dinámicas, evaluar el seguimiento de trayectoria (*tracking error*), afinar las ganancias del corrector de cascada y graficar el rechazo de perturbaciones (backlash) sin afectar en lo absoluto el rendimiento en tiempo real del motor.

---

## Conclusión

El firmware resultante es una plataforma de control determinista y a prueba de fallos. El uso inteligente del procesamiento dual-core garantiza estabilidad motriz, mientras que la integración del encoder magnético con un anillo multipolar eleva la resolución a niveles metrológicos sin sacrificar velocidad. Finalmente, la canalización de telemetría sin bloqueo nos provee de las herramientas de introspección necesarias para validar la precisión del proyecto.
