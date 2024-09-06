# Comunicación I2C

### Conexiones para la prueba con Arduino Nano

    1. GND (STM32) ---> GND (NANO)
    2. 5V  (STM32) ---> R_PULL_UP1 y R_PULL_UP2
    3. PB6 (STM32) ---> R_PULL_UP1 ---> PA5 (NANO) [SCL]
    4. PB7 (STM32) ---> R_PULL_UP2 ---> PA4 (NANO) [SDA]