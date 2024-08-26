# VSC + Docker + PlatformIO + STM32
En base a [este artículo](https://www.linkedin.com/pulse/connecting-microcontrollers-platformio-running-linux-windows-jenssen-c6ulf/)
Más info: [tutorial de Microsoft](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
---

### 1. Introducción

Para el desarrollo, es necesario acceder a dispositivos USB que están físicamente conectados al sistema operativo host (Windows) y hacer que estos dispositivos sean accesibles dentro del contenedor Docker que ejecuta el entorno de desarrollo en Linux. La solución propuesta consiste en:

1. EL entorno de desarrollo corre dentro de un contenedor Docker con un SO Linux
2. El dispositivo USB conectado en Windows es compartido y reenviado al entorno Linux dentro de WSL
3. Ese dispositivo es accesible dentro del contenedor Docker
4. Usando Dev Containers de VSC se puede interactuar con este entorno desde Windows, usando la interfaz gráfica de VSC, pero ejecutando el código en un contenedor Linux, con acceso a los dispositivos USB necesarios

### 2. Configuraciones
1. Instalar usbipd (disponible [acá](https://github.com/dorssel/usbipd-win/releases) o en la carpeta Drivers)
2. Abrir PowerShell (WIN) con permisos de **administrador** y ejecutar `usbipd list`
3. Observar el bus ID de STM32 STLink (por ejemplo, 1-1)
4. Abrir Docker e iniciar el contenedor
5. En PowerShell (WIN), ejecutar `usbipd bind --busid 1-1` (modificar 1-1 por el bus ID)
6. Ejecutar (PowerShell - WIN) `usbipd attach --wsl --busid 1-1` (modificar 1-1 por el bus ID). Es importante que el contenedor esté corriendo
7. Iniciar el Dev Containers de VSC y adjuntar el contenedor actual
8. En la terminal (VSC - LIN), ejecutar `apt-get update && apt-get install usbutils`
9. Comprobar con `lsusb` (terminal VSC - LIN) que aparezca STMicroelectronics STM32 STLink
10. Agregarle al `platformio.ini` la línea

    `upload_flags = -c set CPUTAPID 0x2ba01477`

    Más info: [nota de la comunidad](https://community.platformio.org/t/debugging-of-stm32f103-clone-bluepill-board-wrong-idcode/14635)

OBS: cada vez que se deconecte el ST-Link debe ejecutarse en PowerShell (WIN) `usbipd attach --wsl --busid 1-1` (modificar 1-1 por el bus ID)
