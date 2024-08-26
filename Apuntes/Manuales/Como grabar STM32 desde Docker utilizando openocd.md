# Grabado desde Docker SIN utilizar PlatformIO
---

### 1. Prerrequisitos

Se asume que se configuró el configuró el contenedor mediante el Dockerfile con las siguientes características:

1. Instalación de openocd
2. Reemplazo del archivo *stm32f1x.cfg* de la carpeta *scripts/target* de openocd por el que se encuentra en el repositorio o bien la modificación del bloque

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

3. La creación por bashrc del alias *program_stm32*:
   ```
    RUN echo "program_stm32() { \
        if [ \"\$#\" -ne 2 ]; then \
            echo \"Usage: program_stm32 <file> <address>\"; \
            return 1; \
        fi; \
        local file=\"\$1\"; \
        local address=\"\$2\"; \
        openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c \"program \$file \$address\" -c \"shutdown\"; \
    }" >> ~/.bashrc
   ```

### Uso:

  1. En Windows, vincular el puerto USB donde se conectó el ST-LINK V2 (modificando 1-1 por el bus ID):

     `usbipd attach --wsl --busid 1-1`

  2. Ejecutar el contenedor
  3. En la carpeta donde esté el archivo binario que se quiera grabar, ejecutar:

     `program_stm32 <nombre-archivo.bin> <memoria>`

      donde en 'memoria' estamos usando 0x08000000
     
Obs: si no se creó el alias para *program_stm32*, se debe ejecutar:

    `openocd -f /usr/share/openocd/scripts/interface/stlink.cfg -f /usr/share/openocd/scripts/target/stm32f1x.cfg -c "program <nombre-programa> <memoria>" -c "shutdown"`
