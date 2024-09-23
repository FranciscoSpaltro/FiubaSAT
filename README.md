# Comunicación I2C

### Conexiones para la prueba con Arduino Nano

    1. GND (STM32) ---> GND (NANO)
    2. 5V  (STM32) ---> R_PULL_UP1 y R_PULL_UP2
    3. PB6 (STM32) ---> R_PULL_UP1 ---> PA5 (NANO) [SCL]
    4. PB7 (STM32) ---> R_PULL_UP2 ---> PA4 (NANO) [SDA]

### Próximos pasos
- Identificación de tipo de dato en el handshake? Vale la pena? El resto de los sensores no van a tener un micro
- Protección ante dirección de esclavo repetida
- Fallo en la conexión cableada se cuelga el I2C?
- En una falla que pasa con los mensajes que no se enviaron?
- Funciona bien la conexión entre hacer el rq (poner el mensaje en tx) y leer el dato (poner el mensaje en rx)? Es decir quien envia el dato puede trackear la respuesta?
- Investigar funcion i2c_write_restart de ve3wwg

- Tema memoria
- Si pido 5 bytes y me manda 1 que no se quede esperando después de un tiempo

### Pruebas
- [ ] Desconectar esclavo y reconectar
- [ ] Enviar y recibir intercalado


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