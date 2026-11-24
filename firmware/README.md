# Firmware del eje (fuente)

Proyecto de [PlatformIO](https://platformio.org/) para la Raspberry Pi Pico (RP2040), con el core de Arduino. Para usar el eje **no hace falta compilarlo**: el paquete de Python trae el `.uf2` listo y se graba con `umachining-flashear` (ver el [README principal](../README.md#3-el-firmware-de-la-placa)). Esta carpeta es para quien quiera cambiar el firmware.

## Compilar y grabar

```bash
pip install platformio
cd firmware
pio run                 # compilar: queda en .pio/build/pico/firmware.uf2
pio run -t upload       # compilar y grabar por USB (cerrar antes la interfaz y cualquier monitor serie)
pio device monitor      # consola serie a 921 600 baudios
```

La primera compilación baja la plataforma y el compilador (unos cientos de MB) y tarda varios minutos.

En Linux, para que `pio run -t upload` pueda grabar, hay que instalar una vez las reglas udev de PlatformIO:

```bash
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Si la grabación no arranca, poner la placa en modo BOOTSEL a mano (desenchufar, apretar BOOTSEL, enchufar) y volver a correr `pio run -t upload`, o grabar el `.uf2` con `umachining-flashear --uf2 .pio/build/pico/firmware.uf2`.

**Pico 2 (RP2350):** en `platformio.ini`, cambiar `board = rpipico` por `board = rpipico2`. Esa variante no se probó en el eje.

## Dónde está cada cosa

| Archivo | Qué tiene |
|---|---|
| `src/hw_config.h` | Pines, geometría del montaje (husillo, rueda de imanes), límites de recorrido y velocidad, corriente del motor y ganancias del PID |
| `src/main.cpp` | Arranque, lazo principal y telemetría |
| `src/command_processor.cpp` | Los comandos G-code / M-code |
| `src/axis_controller.cpp` | Homing, calibración y estado del eje |
| `src/servo_control/` | Lazo en cascada posición → velocidad → campo, anti-cogging, calibración del encoder |
| `src/hardware/` | Drivers del encoder MT6835 y del puente TB6612 |
| `src/motion_profile/` | Perfil de velocidad trapezoidal |

La [arquitectura del firmware](../docs/DOCUMENTACION_ARQUITECTURA.md) explica cómo se reparten las tareas entre los dos núcleos.

Las ganancias del PID se pueden probar sin recompilar, con `M55` (por ejemplo desde la consola de la interfaz). Solo se pasan a `hw_config.h` una vez elegidas.

## Actualizar el firmware que trae el paquete

Después de cambiar algo en `src/`:

```bash
python firmware/empaquetar.py
```

Compila, copia el `.uf2` a `umachining/firmware/` y actualiza su SHA-256. Hay que actualizar a mano `umachining/firmware/INFO.txt` y commitear los tres archivos, así `pip install` instala el firmware nuevo.
