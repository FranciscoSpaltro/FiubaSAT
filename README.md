# Comunicación I2C

### Conexiones para la prueba con Arduino Nano

    1. GND (STM32) ---> GND (NANO)
    2. 5V  (STM32) ---> R_PULL_UP1 y R_PULL_UP2
    3. PB6 (STM32) ---> R_PULL_UP1 ---> PA5 (NANO) [SCL]
    4. PB7 (STM32) ---> R_PULL_UP2 ---> PA4 (NANO) [SDA]

### Tareas
- Rutina para chequeo de salud de los dispositivos
- Sacar los taskDelay, ver xTaskGetTickCount()

### Observaciones
- En los casos de reconexión/retransmisión, ver de combinar el taskYIELD() con un xTaskGetTickCount() para hacer que no vuelva a probar hasta que pase determinado tiempo
- Vale la pena las interrupciones?
- Revisar la variable global con la dirección del esclavo -> si hay más de un request como hago??