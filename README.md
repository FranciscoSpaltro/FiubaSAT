# Comunicación I2C

### Conexiones para la prueba con Arduino Nano

    1. GND (STM32) ---> GND (NANO)
    2. 5V  (STM32) ---> R_PULL_UP1 y R_PULL_UP2
    3. PB6 (STM32) ---> R_PULL_UP1 ---> PA5 (NANO) [SCL]
    4. PB7 (STM32) ---> R_PULL_UP2 ---> PA4 (NANO) [SDA]

### Protocolo para imprimir mensajes por Arduino via I2C
- Maestro (STM32): sin interrumpir la comunicación
    - Inicia la comunicación
    - Envía un comando a definir (IMPRIMIR_MENSAJE)
    - Envía la cadena caracteres sin interrumpir
    - Finaliza la comunicación
- Esclavo (ARDUINO):
    - Lee el comando
    - Recibe todos los bytes
    - Una vez finalizada la comunicacióon, imprime

### OBSERVACIONES
- FreeRTOS utiliza un contador de 32 bits para contar los ticks del sistema, lo que significa que se desbordará después de aproximadamente 49.7 días (para un sistema que corre a 1 ms por tick). Sin embargo, el cálculo de la diferencia, como en tu código, maneja correctamente el desbordamiento debido a la aritmética de enteros sin signo

### Próximos pasos
- Identificación de tipo de dato en el handshake? Vale la pena? El resto de los sensores no van a tener un micro
- Protección ante dirección de esclavo repetida
- Fallo en la conexión cableada se cuelga el I2C?
- En una falla que pasa con los mensajes que no se enviaron?
- Funciona bien la conexión entre hacer el rq (poner el mensaje en tx) y leer el dato (poner el mensaje en rx)? Es decir quien envia el dato puede trackear la respuesta?
- Investigar funcion i2c_write_restart de ve3wwg

- Tema memoria
- Si pido 5 bytes y me manda 1 que no se quede esperando después de un tiempo

### Comandos útiles RPi5
- pinout: muestra el pinout de la RPi
- gpiodetect: muestra los chips
- pinctrl: muestra info de todos los pines
- gpioinfo: muestra info de todos los pines por chip
- gpioget gpiochip0 4: muestra el estado del pin 4 del chip 0


### Tareas
- Rutina para chequeo de salud de los dispositivos
- Sacar los taskDelay, ver xTaskGetTickCount()
- ¿Mapa de esclavos por puerto? Y que decida por donde enviarlo (solo se necesitaría el slave_addr)

### Observaciones
- En los casos de reconexión/retransmisión, ver de combinar el taskYIELD() con un xTaskGetTickCount() para hacer que no vuelva a probar hasta que pase determinado tiempo
- Vale la pena las interrupciones?
- Revisar la variable global con la dirección del esclavo -> si hay más de un request como hago??
- La dirección 0x00 con el mensaje 0 está reservada para ERROR
- Función para leer?
- Revisar make_request, siempre hace get_i2c


### PROCESO DE TRANSMISION DE UN MENSAJE
1. Encolar un msg_t con la dirección y el dato a enviar en TXQ. El flag request debe estar en false
2. Iniciar la comunicación I2C en modo WRITE con i2c_start (puerto I2C, dirección del esclavo, false)
3. Enviar el byte de datos con i2c_write (puerto I2C, byte de datos)
4. Esperar a que se complete la transferencia de datos
5. Comprobar si se recibió ACK del esclavo
6. Enviar un STOP para finalizar la comunicación

### PROCESO DE SOLICITUD Y RECEPCIÓN DE UN MENSAJE
1. Encolar un msg_t con la dirección del esclavo en TXQ. El flag request debe estar en true
2. Iniciar la comunicación I2C en modo READ con i2c_start (puerto I2C, dirección del esclavo, true)
3. Leer el byte de datos con i2c_read (puerto I2C, true) y encolarlo en RXQ
4. Enviar un NACK si es el último byte a leer
5. Enviar un STOP para finalizar la comunicación

### FUNCIONES LIBOPENCM3
#### FUNCIONES DE CONFIGURACIÓN
1. i2c_peripheral_enable(uint32_t i2c): habilita el periférico I2C

2. i2c_peripheral_disable(uint32_t i2c): deshabilita el periférico I2C

3. i2c_set_own_7bit_slave_address(uint32_t i2c, uint8_t slave): configura la dirección de 7 bits para el periférico cuando está en modo esclavos

4. i2c_set_own_10bit_slave_address(uint32_t i2c, uint8_t slave): análogo pero para una dirección de 10 bits

5. i2c_enable_dual_addressing_mode(uint32_t i2c): habilita el modo de direccionamiento dual del periférico para permitir que el esclavo escuche dos direcciones

6. i2c_disable_dual_addressing_mode(uint32_t i2c): deshabilita el modo de direccionamiento dual

7. i2c_set_own_7_bit_slave_address_two(uint32_t i2c, uint8_t slave): configura una segunda dirección de esclavo de 7 bits

8. i2c_set_clock_frequency(uint32_t i2c, uint8_t freq): establece la frecuencia del reloj del periférico I2C en MHz

9. i2c_set_fast_mode(uint32_t i2c): configura el periférico para operar en modo rápido (hasta 400 kHz)

10. i2c_set_standard_mode(uint32_t i2c): configura el periférico para operar en modo estándar (hasta 100 kHz)

11. i2c_set_ccr(uint32_t i2c, uint16_t ccr): configura el valor del registro CCR (Clock Control Register) para controlar la velocidad de transmisión en el bus

12. i2c_set_trise(uint32_t i2c, uint8_t trise): establece el tiempo máximo de subida para la señal I2C (en ciclos de reloj)

13. i2c_set_dutycycle(uint32_t i2c, uint16_t dutycycle): configura el ciclo de trabajo del reloj I2C en modo rápido

14. i2c_enable_interrupt(uint32_t i2c, uint32_t interrupts): habilita interrupciones específicas para el periférico

15. i2c_disable_interrupt(uint32_t i2c, uint32_t interrupts): deshabilita interrupciones específicas para el periférico

16. i2c_enable_ack(uint32_t i2c): habilita la generación de señales de reconocimiento en respuesta a la recepción de datos

17. i2c_disable_ack(uint32_t i2c): deshabilita la generación de señales ACK, de modo que no se reconozcan las transmisioines entrantes. Se utiliza cuando se necesita ignorar las transmisiones o enviar una señal de NACK

18. i2c_enable_dma(uint32_t i2c): habilita el uso de DMA (Direct Memory Access) para las transmisiones I2C, lo que permite la transferencia automática de datos sin intervención del procesador (se utiliza cuando hay que realizar transferencias grandes de datos con mayor eficiencia)

19. i2c_disable_dma(uint32_t i2c): deshabilita el uso de DMA para las transmisiones I2C

20. i2c_set_speed(uint32_t i2c, uint32_t clock, uint32_t speed): configura la velocidad del bus I2C según la frecuencia de entrada y la velocidad de transferencia deseada (estándar o rápido)


#### FUNCIONES OPERATIVAS

1. i2c_send_start(uint32_t i2c): envia una condicion de inicio a los esclavos

2. i2c_get_data(uint32_t i2c): recupera un byte de datos del registro de datos

3. i2c_send_data(uint32_t i2c, uint8_t data): envía un byte de datos a través del bus I2C

4. i2c_send_stop(uint32_t i2c): envia una condicion de parada en el bus I2C

5. i2c_clear_stop(uint32_t i2c): borra la señal de STOP, permitiendo que el periférico vuelva a un estado donde pueda iniciar nuevas operaciones de transmisión

6. i2c_nack_next(uint32_t i2c): envía un NACK después del próximo byte recibido. Se utiliza cuando el maestro no necesita más datos del esclavo y quiere terminar la comunicación

7. i2c_nack_current(uint32_t i2c): envía un NACK  inmediatamente después de recibir el byte actual para finalizar la recepción de datos

8. i2c_set_dma_last_transfer(uint32_t i2c): indica que la próxima transmisión será la úlitma en una secuencia de transferencias DMA

9. i2c_clear_dma_last_transfer(uint32_t i2c): borra el flag que indica la última transferencia DMA

10. [STATIC] i2c_read7_v1(uint32_t i2c, uint8_t addr, uint8_t *buf, size_t len): realiza una lectura desde un dispositivo esclavo de 7 bits y almacena los datos en el buffer proporcionado

11. [STATIC] i2c_write_7_v1(uint32_t i2c, uint8_t addr, const uint8_t data, size_t length): envía una secuencia de datos al dispositivo esclavo de direccion de 7 bits en el bus I2C

12. i2c_transfer7(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn): realiza una transacción completa I2C de 7 bits con escritura y lectura (si corresponde). Se envía datos y luego se reciben en la misma transacción
