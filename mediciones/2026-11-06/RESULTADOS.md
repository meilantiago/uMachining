# Pruebas del 06/11/2026: ruido en reposo con el motor acoplado

Firmware en la placa: build del 03/11 17:39 del árbol de `prueba/acoplado-velocidad` (factor de corriente 0,6, `VEL_KP` 0,12, `VEL_KI` 54; las constantes se verificaron en el `.uf2`).

## Problema

El 03/11 (`../2026-11-03/escalera_171740.csv`), a 30 mm, el reposo tenía 27 nm de desvío y los escalones de 10 nm, entre 3 y 18 nm. El 23/10 eran 1,4 nm. Era una oscilación de banda angosta de ~205 Hz, no ruido blanco. Su amplitud era constante dentro de cada escalón y cambiaba de uno a otro, y la posición y la salida del lazo tenían correlación −0,99.

## 1. El encoder no es: motor apagado (`reposo_ganancias/off_P*.npz`)

- Ruido muestra a muestra: 0,6–0,9 nm. Ningún bit de estado (campo débil, overspeed, subtensión). No hay nada entre 150 y 260 Hz.
- Hay una vibración de ~3 nm a 25 Hz con el rotor libre. Es mecánica o ambiental y con el lazo cerrado baja a <1 nm.
- La configuración del chip (BW 5, histéresis 4, SPI a 8 MHz) no cambió desde el 16/10. Ningún firmware escribe la EEPROM del MT6835: `M122` de `motor_libre` cambia BW e histéresis solo en RAM, y `axis_controller.cpp` los vuelve a 5/4 en cada arranque.

## 2. La tabla de calibración en flash no correspondía a la mecánica

- Al arrancar se carga la tabla de flash. Después de `G28`, el desfase de conmutación era de 36,5° eléctricos, contra −8…+14° el 03/11.
- Un `G1 X30` cortó por falla. Ya a ~1 mm/s la salida estaba en 58–77°, se saturó en 81° y el carro se clavó en 0,554 mm (`traza_corte_tabla_flash.csv`).
- `M57 X5` (`m57_tabla_flash.log`) dio +73 ± 18° (de +26 a +142°, con patrón de una vez por vuelta), cuando el 03/11 daba +30 ± 3°.
- Después de `M56` (sin `S`, `m56.log`): conmutación −5,2°, anti-cogging con RMS de 0,87 µm. `M57` desde 0 (`m57_postcal.log`): +30,2 ± 3,7°, igual que el 03/11. **La rueda no se corrió; la tabla de flash es vieja** (probablemente anterior al reacople).
- **La calibración nueva está solo en RAM.** Si la placa se reinicia, vuelve la tabla mala.

## 3. El ruido de ~214 Hz lo produce `VEL_KP` (`reposo_ganancias/`)

Reposo a 30 mm, recién calibrado, ganancias cambiadas con `M55 A60 B100 C<kp> D<ki> F0.004`:

| `VEL_KP` | `VEL_KI` | Desvío | Potencia en 150–260 Hz |
|---|---|---|---|
| 0,12 | 54 | 2,0–2,3 nm | 23 % |
| 0,12 | 90 | 2,8 nm | 15 % |
| 0,16 | 54 | 5,1 nm | 77 % |
| 0,16 | 72 | 9,5 nm | 91 % |
| 0,20 | 54 | 10,6 nm | 93 % |
| 0,20 | 90 (master) | 10,8–11,1 nm | 92–96 % |
| 0,24 | 54 | 11,7 nm | 94 % |

- A 2 kHz el pico sigue en 214 Hz: es real, no un alias.
- `VEL_KI` casi no influye. El umbral está entre 0,12 y 0,16 con factor 0,6. Con el motor reacoplado, el lazo de velocidad de master (0,2/90) queda al borde de la inestabilidad en ~214 Hz. El 03/11 a las 17:17 la placa todavía tenía esas ganancias; el build con 0,12/54 es de las 17:39.

## 4. Resultado con tabla nueva y `VEL_KP` 0,12 / `VEL_KI` 54

- Escalera de 10 nm a 30 mm (`escalera_092713.csv`): reposo con 1,59 nm de desvío; saltos de +10,34, +9,78, +9,95, +9,97 y +10,00 nm; desvío por escalón de 1,5–2,1 nm.
- 0 → 50 → 0 a 2 mm/s (`lazo_cerrado_20261106_093001_postcal_kp012`): RMS de 0,9 / 1,0 µm y máximo de 3,0 / 3,5 µm, con la salida a 45° como máximo. El 03/11 eran 1,7 µm, 6,3 µm y 62°.

## Pendiente

- Guardar la calibración en flash con `M56 S`.
- Llevar `VEL_KP` 0,12 / `VEL_KI` 54 a master con el factor 0,6. El comentario actual de `hw_config.h` dice que se escalaron por el paso a factor 1,0, y eso ya no es cierto. Agregar la entrada al `CHANGELOG.md`.
- Repetir la escalera en 10, 100 y 200 mm. Hoy solo se midió a 30 mm.
