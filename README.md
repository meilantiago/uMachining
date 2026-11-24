# μMachining: guía de instalación

Control en lazo cerrado de un eje lineal de micromaquinado: un motor paso a paso con un husillo de bolas, un encoder magnético que cierra el lazo y una Raspberry Pi Pico (RP2040) que hace el control. Este paquete instala en cualquier PC la **interfaz gráfica** que maneja el eje y la herramienta para **grabar el firmware** en la placa.

**Autores:** Francisco Maresca · Tiago Meilán

## Qué se instala

| Comando | Para qué |
|---|---|
| `umachining` | Interfaz gráfica: control remoto del eje y telemetría en vivo ([guía de uso](docs/GUIA_INTERFAZ.md)) |
| `umachining-gcode` | Mandar comandos G-code desde la terminal |
| `umachining-flashear` | Grabar el firmware en la Pico (el `.uf2` ya compilado viene dentro del paquete) |

Las dependencias (`numpy`, `matplotlib`, `pyserial`, `PyQt5`, `pyqtgraph`) se instalan solas. No hace falta PlatformIO salvo que se quiera modificar y recompilar el firmware.

## Requisitos

- Python 3.8 o más nuevo, en Windows, macOS o Linux (PC común, x86-64).
- Un cable USB **de datos** (algunos cables solo cargan, y con esos la placa no aparece).
- El eje armado, con la Pico conectada al driver, al motor y al encoder. Los pines están en [`firmware/src/hw_config.h`](firmware/src/hw_config.h).

Para ver si Python ya está instalado: `python3 --version` (en Windows, `py --version`). Si no está, se baja de [python.org](https://www.python.org/downloads/). En Windows, marcar **Add python.exe to PATH** en el instalador.

## 1. Instalar el paquete

### Con el instalador (recomendado)

1. Bajar el repositorio: [Firmware-para-usuarios.zip](https://github.com/meilantiago/uMachining/archive/refs/heads/Firmware-para-usuarios.zip), y descomprimirlo.
2. Correr el instalador:
   - **Windows:** doble clic en `instalar.bat`.
   - **Linux y macOS:** abrir una terminal en la carpeta descomprimida y correr `python3 instalar.py`.

El instalador va mostrando cada paso (revisa Python, crea el entorno, instala las dependencias y revisa los permisos del puerto USB) y al final avisa **Instalado correctamente** y muestra cómo abrir la interfaz. Todo queda en la carpeta `umachining-venv` dentro de la carpeta personal; para usar otra: `python3 instalar.py --entorno <carpeta>`. Volver a correrlo actualiza la instalación.

### A mano, con pip

Se instala en un entorno virtual, para no mezclarlo con el Python del sistema.

**Linux y macOS**

```bash
python3 -m venv ~/umachining-venv
source ~/umachining-venv/bin/activate
pip install https://github.com/meilantiago/uMachining/archive/refs/heads/Firmware-para-usuarios.zip
```

**Windows** (en «Símbolo del sistema»)

```bat
py -m venv %USERPROFILE%\umachining-venv
%USERPROFILE%\umachining-venv\Scripts\activate
pip install https://github.com/meilantiago/uMachining/archive/refs/heads/Firmware-para-usuarios.zip
```

En una terminal nueva, antes de usar los comandos hay que volver a activar el entorno (la segunda línea).

> **Alternativa sin activar nada:** con [pipx](https://pipx.pypa.io/) los comandos quedan disponibles en cualquier terminal:
> `pipx install https://github.com/meilantiago/uMachining/archive/refs/heads/Firmware-para-usuarios.zip`

**Desde una copia del repositorio** (para modificar el código):

```bash
git clone -b Firmware-para-usuarios https://github.com/meilantiago/uMachining.git
cd uMachining
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
```

**Comprobar la instalación:**

```bash
umachining-flashear --info
```

Muestra el firmware que trae el paquete y las placas que encuentra conectadas.

## 2. Permisos del puerto serie (solo Linux)

En Linux, un usuario común no puede abrir el puerto USB de la placa hasta que lo agregan al grupo `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Después hay que **cerrar la sesión y volver a entrar** (o reiniciar). El instalador revisa esto y ofrece correr el comando. En Windows 10/11 y en macOS no hace falta instalar drivers ni dar permisos.

## 3. El firmware de la placa

### 3a. La placa ya tiene el firmware

Es el caso normal: las placas del eje se entregan grabadas. No hay que hacer nada. Para comprobar que responde:

```bash
umachining-gcode M119
```

Tiene que imprimir el estado del eje y terminar en `ok`. Si es así, pasar al punto 4.

### 3b. La placa no tiene firmware (o hay que reemplazarlo)

Con la placa conectada por USB:

```bash
umachining-flashear
```

- Si la placa ya tiene un firmware de este proyecto, el programa pide confirmación y la pasa sola a modo de grabación.
- Si es una Pico nueva, o no responde, el programa pide ponerla en modo de grabación a mano y espera:
  1. Desenchufar el USB de la Pico.
  2. Mantener apretado el botón blanco **BOOTSEL** de la placa.
  3. Enchufar el USB y soltar el botón.

En ese modo la Pico aparece como una unidad USB llamada `RPI-RP2`. El programa copia el firmware, la placa se reinicia sola y queda lista.

**Sin el programa:** también se puede hacer a mano. Poner la placa en modo BOOTSEL como arriba y arrastrar el archivo `firmware.uf2` a la unidad `RPI-RP2`. `umachining-flashear --info` dice dónde quedó instalado ese archivo, y también está en [`umachining/firmware/`](umachining/firmware/) de este repositorio.

> Grabar el firmware reinicia la placa: la calibración se pierde y hay que volver a calibrar (punto 4).
> El firmware compilado es para la **Pico original (RP2040)**. Para una Pico 2 (RP2350) hay que compilarlo desde el fuente.

### 3c. Modificar y recompilar el firmware

Solo si se quieren cambiar parámetros del montaje o del control (`firmware/src/hw_config.h`). Ver [`firmware/README.md`](firmware/README.md).

## 4. Primer uso

```bash
umachining
```

La interfaz encuentra la placa y se conecta sola. **Cada vez que se enciende la placa**, antes de mover:

1. Dejar libre todo el recorrido (0 a 255 mm).
2. Apretar **Calibrar e iniciar (M56)** y confirmar. Tarda unos 3 minutos: el carro va al tope de 0 mm, calibra el encoder y va y vuelve un par de veces.
3. Cuando los cuatro indicadores de arriba quedan en verde, ya se puede mover el carro.

La [guía de la interfaz](docs/GUIA_INTERFAZ.md) explica cada control. Las grabaciones de telemetría y los barridos del encoder se guardan en `~/uMachining/mediciones/AAAA-MM-DD/` (en Windows, `C:\Users\<usuario>\uMachining\mediciones\`). Para usar otra carpeta: `umachining --datos <carpeta>`, o la variable de entorno `UMACHINING_DATOS`.

**Desde la terminal**, sin la interfaz:

```bash
umachining-gcode "M56" "M17" "G1 X10 F2" "M114" --timeout 600
```

| Comando | Qué hace |
|---|---|
| `M56` | Homing y calibración (correrlo en cada encendido) |
| `M17` / `M18` | Habilitar el lazo / apagar el motor |
| `G1 X<mm> [F<mm/s>]` | Mover a una posición (0 a 255 mm). La velocidad no pasa del tope, que arranca en 2 mm/s |
| `M203 X<mm/s>` | Cambiar el tope de velocidad (hasta 5 mm/s; el anti-cogging está aprendido para 2 mm/s) |
| `G90` / `G91` | Posiciones absolutas / relativas |
| `M114` / `M119` | Posición / estado completo |
| `M55 …` | Cambiar las ganancias del PID sin regrabar |
| `M120 S1 P<n>` | Telemetría binaria (la usa la interfaz) |
| `M121` | Traza del último corte por falla |

La placa usa 921 600 baudios. Solo un programa por vez puede tener abierto el puerto: cerrar la interfaz antes de usar `umachining-gcode` o un monitor serie, y al revés.

## 5. Si algo no funciona

| Problema | Qué hacer |
|---|---|
| `error: externally-managed-environment` al hacer `pip install` | Falta el entorno virtual: seguir el punto 1 tal cual, o usar pipx. |
| `umachining: command not found` | El entorno virtual no está activado en esa terminal (`source ~/umachining-venv/bin/activate`). |
| La interfaz no abre en Linux: `Could not load the Qt platform plugin "xcb"` | Faltan bibliotecas del sistema: `sudo apt install libxcb-xinerama0 libxcb-cursor0 libxkbcommon-x11-0` |
| **No se pudo abrir el puerto** / `Permission denied` | Otro programa tiene el puerto abierto (otra interfaz, monitor serie de PlatformIO, Arduino IDE): cerrarlo. En Linux, revisar el punto 2. |
| La placa no aparece en la lista de puertos | Probar otro cable USB (de datos) y otro puerto de la PC. `umachining-flashear --info` muestra lo que se detecta. |
| `umachining-flashear` no encuentra la unidad `RPI-RP2` | Hacer el BOOTSEL a mano (punto 3b). En Linux sin escritorio la unidad no se monta sola: montarla (`sudo mount /dev/sdX1 /mnt`) y correr `umachining-flashear --unidad /mnt`. |
| Raspberry Pi u otra PC ARM con Linux | PyQt5 no se instala desde pip ahí: `sudo apt install python3-pyqt5 python3-pyqtgraph`, crear el entorno con `python3 -m venv --system-site-packages ~/umachining-venv` y seguir el punto 1. |
| **Corte por falla**, eje sin calibrar, lazo apagado | Ver «Si algo sale mal» en la [guía de la interfaz](docs/GUIA_INTERFAZ.md#8-si-algo-sale-mal). |

## Actualizar y desinstalar

```bash
pip install --upgrade --force-reinstall https://github.com/meilantiago/uMachining/archive/refs/heads/Firmware-para-usuarios.zip
pip uninstall umachining
```

Actualizar el paquete no regraba la placa: para eso, `umachining-flashear`. Desinstalar no borra las mediciones de `~/uMachining/`.

## Contenido del repositorio

```
umachining/          paquete de Python: interfaz, herramientas y firmware compilado
  firmware/          firmware.uf2 listo para grabar, su SHA-256 y de qué versión es
firmware/            fuente del firmware (proyecto de PlatformIO)
  src/hw_config.h    pines, parámetros del montaje y ganancias del control
docs/                guía de la interfaz, arquitectura del firmware y origen del código
instalar.py          instalador (instalar.bat lo abre con doble clic en Windows)
setup.cfg            definición del paquete para pip (pyproject.toml solo elige setuptools)
```

## Hardware

| | |
|---|---|
| Placa | Raspberry Pi Pico (RP2040) |
| Motor | Paso a paso de 1,8° (50 pares de polos) |
| Driver | TB6612 |
| Transmisión | Husillo de bolas de 5 mm por vuelta, 255 mm de recorrido |
| Encoder | MT6835 de 21 bits frente a una rueda de 60 imanes |

## Créditos

El firmware está basado en `MotionControllerRP` de [MicroManipulatorStepper](https://github.com/0x23/MicroManipulatorStepper) (licencia MIT). Ver [qué se reutilizó y qué se reescribió](docs/README_ORIGEN_DEL_CODIGO.md) y la [arquitectura del firmware](docs/DOCUMENTACION_ARQUITECTURA.md).
