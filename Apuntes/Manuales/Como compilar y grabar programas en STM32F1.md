### 1. Introducción

Para el desarrollo, es necesario acceder a dispositivos USB que están físicamente conectados al sistema operativo host (Windows) y hacer que estos dispositivos sean accesibles dentro del contenedor Docker que ejecuta el entorno de desarrollo en Linux. La solución propuesta consiste en:

1. EL entorno de desarrollo corre dentro de un contenedor Docker con un SO Linux
2. El dispositivo USB conectado en Windows es compartido y reenviado al entorno Linux dentro de WSL
3. Ese dispositivo es accesible dentro del contenedor Docker
4. Usando Dev Containers de VSC se puede interactuar con este entorno desde Windows, usando la interfaz gráfica de VSC, pero ejecutando el código en un contenedor Linux, con acceso a los dispositivos USB necesarios

---

### 2. Prerrequisitos
1. Tener el ST-LINK V2 compartido y asociado al contenedor
   
- **Opción 1:** ejecutar `./attach_usb`

- **Opción 2:** realizar el bind y/o attach (según corresponda) de forma manual. Ver instrucciones en [Como construir y ejecutar el Docker](https://github.com/fran855/FiubaSAT/blob/5e3dd43a80d1b997840d32e1bd523a2408d4fb46/Apuntes/Manuales/Como%20construir%20y%20ejecutar%20el%20Docker.md)

   > Se puede comprobar la asociación utilizando el comando `lsusb` en WSL, habiendo instalado previamente el paquete *usbutils* [`apt-get update && apt-get install usbutils`]

2. Si se quiere realizar las acciones desde PlatformIO, agregar al archivo *platformio.ini* la línea

    `upload_flags = -c set CPUTAPID 0x2ba01477`

    Más info: [nota de la comunidad](https://community.platformio.org/t/debugging-of-stm32f103-clone-bluepill-board-wrong-idcode/14635)

3. Reemplazar el archivo .cfg de openocd para admitir el grabado en Blue Pill genéricas:
  - **Si se construyó el Docker a partir del Dockerfile de este repositorio, no debe hacerse nada**
  - Caso constrario, realizar una de las siguientes acciones:
    
    i. Reemplazar el archivo *stm32f1x.cfg* de la carpeta /usr/share/openocd/scripts/target/stm32f1x.cfg por el que se encuentra en el repositorio (FiubaSAT/Drivers)
    ii. Reemplazar el bloque del archivo *stm32f1x.cfg* original:
    
       ```
          #jtag scan chain
          if { [info exists CPUTAPID] } {
             set _CPUTAPID $CPUTAPID
          } else {
             if { [using_jtag] } {
                # See STM Document RM0008 Section 26.6.3
                set _CPUTAPID 0x3ba00477
             } {
                # this is the SW-DP tap id not the jtag tap id
                set _CPUTAPID 0x1ba01477
             }
          }
       ```
      
       por
    
       ```
            #jtag scan chain
            if { [info exists CPUTAPID] } {
               set _CPUTAPID $CPUTAPID
            } else {
               if { [using_jtag] } {
                  # See STM Document RM0008 Section 26.6.3
                  set _CPUTAPID 0x3ba00477
               } {
                  # this is the SW-DP tap id not the jtag tap id
                  set _CPUTAPID 0x2ba01477
               }
            }
       ```
---

### Instrucciones
1. Iniciar el contenedor y navegar hasta el Makefile (FiubaSAT/src)
2. Ejecutar el comando `make` para compilar el proyecto. Deben editarse en este Makefile todos los SOURCES necesarios
   
   > Aclaración importante: al clonar este repositorio no se clona la librería libopencm3, indispensable para la compilación del proyecto. El Makefile cuenta con una instrucción adicional que verifica la existencia de este directorio y, si es necesario, lo clona. Luego, compila la librería (específicamente para la placa STM32F1 con el fin de ahorrar tiempo y memoria) y al final comienza a compilar el proyecto

3. Ejecutar el comando `make flash` (habiendo asociado previamente el usb)

   > Observación: la línea necesaria para grabar manualmente es  `openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c "program <nombre-programa> <direccion-memoria>" -c "shutdown"`. El nombre asignado fue *fiubasat* y la dirección de memoria, 0x08000000 

4. Para eliminar todos los archivos objeto y .elf, ejecutar `make clean`. Para eliminar también el binario, ejecutar `make delete`

---
Más información: 
- [VSC + Docker + PlatformIO](https://www.linkedin.com/pulse/connecting-microcontrollers-platformio-running-linux-windows-jenssen-c6ulf/)
- [Conectar USB en WSL](https://learn.microsoft.com/en-us/windows/wsl/connect-usb)
