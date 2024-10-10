#include "i2c.h"

/**************************************** ESTRUCTURAS Y VARIABLES ****************************************/
// Handlers de los puertos I2C
i2c_t i2c1;
i2c_t i2c2;

/**************************************** FUNCIONES PRIVADAS ****************************************/
i2c_status_t enqueue_data(QueueHandle_t queue, uint8_t *data) {
    if (uxQueueSpacesAvailable(queue) == 0) {
        print_uart("Error: La cola está llena. No se puede encolar.\n\r");
        return I2C_FULL_QUEUE;
    }

    if (xQueueSend(queue, data, pdMS_TO_TICKS(10)) != pdPASS) {
        print_uart("Error: No se pudo encolar el mensaje.\n\r");
        return I2C_FAIL;
    }

    return I2C_PASS;
}

BaseType_t dequeue_i2c_response(QueueHandle_t queue, uint8_t *data) {
    if (uxQueueMessagesWaiting(queue) == 0) {
        return I2C_EMPTY_QUEUE;
    }

    if (xQueueReceive(queue, data, pdMS_TO_TICKS(10)) != pdTRUE) {
        print_uart("Error: No se pudo recibir el mensaje.\n\r");
        return I2C_FAIL;
    }

    return I2C_PASS;
}


/**
 * @brief Obtiene el handler de I2C basado en el identificador de 32 bits del periférico
 * 
 * @param i2c_id Identificador del periférico I2C (I2C1 o I2C2)
 * @return Puntero a la estructura que maneja el periférico I2C correspondiente o NULL si no se encuentra
 */

i2c_t * get_i2c(uint32_t i2c_id) {
    switch(i2c_id) {
        case I2C1:
            return &i2c1;
        case I2C2:
            return &i2c2;
        default:
            return NULL;
    }
}


/**
 * @brief Espera hasta que el periférico I2C especificado esté listo
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return bool true si el bus está listo, false si hubo un timeout
 */

bool i2c_wait_until_ready(uint32_t i2c_id) {
    uint32_t start_time = xTaskGetTickCount();  // Obtener el tiempo actual del sistema (FreeRTOS)

    // Espera hasta que el bus esté listo o el timeout expire
    while (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
            // Si el timeout ha sido excedido
            if (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
                // El bus sigue ocupado, indicar fallo
                return false;
            } else {
                // El bus fue liberado correctamente
                return true;
            }
        }
        taskYIELD();  // Ceder el control a otras tareas mientras se espera
    }

    return true;  // Indicar que el bus está listo
}


/**
 * @brief Espera hasta que el bit de start esté establecido
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return bool true si el bus está listo, false si hubo un timeout
 */

bool i2c_wait_until_start(uint32_t i2c_id) {
    TickType_t start_time = xTaskGetTickCount();  // Obtener el tiempo actual del sistema (FreeRTOS)
    TickType_t timeout = pdMS_TO_TICKS(I2C_TIMEOUT_MS);  // Establecer el tiempo de timeout

    // Espera hasta que el bit de start esté establecido o el timeout expire
    while (!(I2C_SR1(i2c_id) & I2C_SR1_SB)) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
            return false;  // Indicar que hubo un timeout
        }
        taskYIELD();  // Ceder el control a otras tareas mientras se espera
    }

    return true;  // Indicar que el bus está listo
}

/**
 * @brief Espera hasta que el bit de address esté establecido
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return bool true si el bus está listo, false si hubo un timeout
 */

bool i2c_wait_until_address(uint32_t i2c_id) {
    TickType_t start_time = xTaskGetTickCount();  // Obtener el tiempo actual del sistema (FreeRTOS)
    TickType_t timeout = pdMS_TO_TICKS(I2C_TIMEOUT_MS);  // Establecer el tiempo de timeout

    // Espera hasta que el bit de start esté establecido o el timeout expire
    while (!(I2C_SR1(i2c_id) & I2C_SR1_ADDR)) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
            return false;  // Indicar que hubo un timeout
        }

        // Verificar si ocurrió un NACK
        if (I2C_SR1(i2c_id) & I2C_SR1_AF) {
            i2c_send_stop(i2c_id); // Enviar un bit de stop
            I2C_SR1(i2c_id) &= ~I2C_SR1_AF; // Limpiar bandera de fallo de ACK
            return false; // Indicar que la comunicación falló
        }

        taskYIELD();  // Ceder el control a otras tareas mientras se espera
    }

    return true;  // Indicar que el bus está listo
}

/**
 * @brief Inicia una comunicación I2C con el esclavo especificado en modo lectura/escritura
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param addr Dirección de 8 bits del esclavo I2C
 * @param read true si se va a leer, false si se va a escribir
 * @return pdPASS si la comunicación fue exitosa, pdFALSE si hubo un error
 */
BaseType_t i2c_start(uint32_t i2c_id, uint8_t addr, bool read) {
    if(!i2c_wait_until_ready(i2c_id))
        return pdFALSE;
        
    i2c_send_start(i2c_id);

    // Esperar hasta que el bit de Start esté establecido
    if (!i2c_wait_until_start(i2c_id)) {
        print_uart("Error: Timeout al esperar el bit de Start.\n\r");
        return pdFALSE;  // Indicar que hubo un timeout
    }
    

    i2c_send_7bit_address(i2c_id, addr, read ? I2C_READ : I2C_WRITE);

    // Esperar hasta que el bit de Address esté establecido
    if (!i2c_wait_until_address(i2c_id)) {
        print_uart("Error: Timeout al esperar el bit de Address.\n\r");
        return pdFALSE;  // Indicar que hubo un timeout
    }

    // Limpiar la bandera de dirección
    (void)I2C_SR2(i2c_id);
    
    return pdPASS; // Indicar que la comunicación fue exitosa
}

/**
 * @brief Envia un byte de datos por I2C en modo WRITE
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param data Byte de datos a enviar
 * @return pdPASS si el byte fue enviado, pdFALSE si hubo un error
 */
BaseType_t i2c_write(uint32_t i2c_id, uint8_t data) {
    TickType_t start_time = xTaskGetTickCount();  // Guardamos el tiempo de inicio
    TickType_t timeout = pdMS_TO_TICKS(I2C_TIMEOUT_MS);  // Establecemos el tiempo de timeout

    i2c_send_data(i2c_id, data);

    // Esperar hasta que se complete la transferencia de datos o el timeout expire
    while (!(I2C_SR1(i2c_id) & I2C_SR1_BTF)) {
        // Verificamos si el timeout ha expirado
        if ((xTaskGetTickCount() - start_time) > timeout) {
            // Timeout: limpiar el registro I2C y devolver error
            return pdFALSE;
        }
        taskYIELD();  // Ceder el control a otras tareas mientras esperamos
    }

    // Comprobar el bit de ACK después de la transferencia
    if (I2C_SR1(i2c_id) & I2C_SR1_AF) {
        // No se recibió ACK del esclavo (NACK)
        // Limpiar la bandera de fallo de ACK
        I2C_SR1(i2c_id) &= ~I2C_SR1_AF;
        // Limpieza opcional del controlador I2C, si es necesario
        return pdFALSE;
    }

    // Verificar el bit de bus error
    if (I2C_SR1(i2c_id) & I2C_SR1_BERR) {
        // Error en el bus: hacer recuperación
        return pdFALSE;
    }

    return pdPASS;
}


/**
 * @brief Lee un byte de datos por I2C en modo READ
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param last true si es el último byte a leer, false si no
 * @return uint8_t Byte de datos leído [REVISAR: tratamiento de error?] (!)
 */
bool i2c_read(uint32_t i2c_id, uint8_t * data, bool last) {
    if (last) {
        i2c_disable_ack(i2c_id);
    } else {
        i2c_enable_ack(i2c_id);
    }


    uint32_t start_time = xTaskGetTickCount();  // Obtener el tiempo actual del sistema (FreeRTOS)

    // Espera hasta que el bus esté listo o el timeout expire
    while (!(I2C_SR1(i2c_id) & I2C_SR1_RxNE)) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
            // Si el timeout ha sido excedido
            if (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
                // El bus sigue ocupado, indicar fallo
                return false;
            } else {
                // El bus fue liberado correctamente
                return true;
            }
        }
        taskYIELD();  // Ceder el control a otras tareas mientras se espera
    }

    *data = i2c_get_data(i2c_id);
    return true;
}



/**************************************** FUNCIONES PUBLICAS ****************************************/

/**
 * @brief Configura y habilita el periférico I2C especificado [REVISAR: habilitacion por separado?] (!)
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return pdPASS si la configuración fue exitosa, pdFALSE si hubo un error
 */
BaseType_t i2c_setup(uint32_t i2c_id) {
    i2c_t * i2c = get_i2c(i2c_id);
    
    if(i2c_id == I2C1){
        i2c -> i2c_id = I2C1;
        rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
        rcc_periph_clock_enable(RCC_I2C1); // Habilitar el reloj para I2C2

        gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA); // Configurar pines para I2C2

        i2c_peripheral_disable(i2c->i2c_id); // Desactivar I2C2 antes de configurar
        rcc_periph_reset_pulse(RST_I2C1);

        i2c_set_standard_mode(i2c->i2c_id); // Configurar modo estándar
        i2c_set_clock_frequency(i2c->i2c_id, I2C_CR2_FREQ_36MHZ); // Configurar frecuencia del reloj a 36 MHz
        i2c_set_trise(i2c->i2c_id, 36); // Configurar tiempo de subida
        i2c_set_dutycycle(i2c->i2c_id, I2C_CCR_DUTY_DIV2); // Configurar ciclo de trabajo
        i2c_set_ccr(i2c->i2c_id, 180); // Configurar CCR para 100 kHz
        i2c_peripheral_enable(i2c->i2c_id); // Habilitar I2C1
    } else if(i2c_id == I2C2){
        i2c -> i2c_id = I2C2;
        rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
        rcc_periph_clock_enable(RCC_I2C2); // Habilitar el reloj para I2C2

        gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C2_SCL | GPIO_I2C2_SDA); // Configurar pines para I2C2

        i2c_peripheral_disable(i2c->i2c_id); // Desactivar I2C2 antes de configurar
        rcc_periph_reset_pulse(RST_I2C1);

        i2c_set_standard_mode(i2c->i2c_id); // Configurar modo estándar
        i2c_set_clock_frequency(i2c->i2c_id, I2C_CR2_FREQ_36MHZ); // Configurar frecuencia del reloj a 36 MHz
        i2c_set_trise(i2c->i2c_id, 36); // Configurar tiempo de subida
        i2c_set_dutycycle(i2c->i2c_id, I2C_CCR_DUTY_DIV2); // Configurar ciclo de trabajo
        i2c_set_ccr(i2c->i2c_id, 180); // Configurar CCR para 100 kHz
        i2c_peripheral_enable(i2c->i2c_id); // Habilitar I2C2
    }
    
    if ((i2c -> mutex = xSemaphoreCreateMutex()) == NULL)
        return pdFALSE;

    if((i2c -> responses = xQueueCreate(10, sizeof(uint8_t))) == NULL)
        return pdFALSE;
    
    return pdPASS;
}


/**
 * @brief Realiza una solicitud de datos al esclavo I2C especificado
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param addr Dirección de 8 bits del esclavo I2C
 * @param length Longitud de los datos a solicitar
 * 
 * @return pdPASS si la solicitud fue exitosa, pdFALSE si hubo un error
 */
BaseType_t i2c_make_request(uint32_t i2c_id, uint8_t addr, size_t length) {
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    /*if(xSemaphoreTake(i2c -> responses_available, pdMS_TO_TICKS(10)) != pdTRUE){
        return pdFALSE;
    }*/
    uint8_t data;
    if (i2c_start(i2c->i2c_id, addr, true) != pdPASS) {
        print_uart("Error al iniciar la comunicación (RQT).\n\r");
        return pdFALSE;
    }
    //taskENTER_CRITICAL(); // Iniciar sección crítica
    // Almacenar los datos recibidos en el buffer estático
    for (size_t i = 0; i < length; i++) {
        if(!i2c_read(i2c->i2c_id, &data, (i == (length - 1)))){ // true solo para el último byte
            print_uart("Error al leer datos (RQT).\n\r");
            return pdFALSE;
        }

        if(enqueue_data(i2c->responses, &data) != I2C_PASS){
            print_uart("Error: No se pudo encolar el mensaje de solicitud.\n\r");
            return pdFALSE;
        }
    }

    i2c_send_stop(i2c->i2c_id);
    //taskEXIT_CRITICAL(); // Finalizar sección crítica>
    

    return pdPASS;
}





/**
 * @brief Envia datos por I2C al esclavo especificado
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param addr Dirección de 8 bits del esclavo I2C
 * @param data Datos a enviar
 * @param length Longitud de los datos a enviar
 * 
 * @return pdPASS si la comunicación fue exitosa, pdFALSE si hubo un error
 */
BaseType_t i2c_send_data_slave(uint32_t i2c_id, uint8_t addr, uint8_t* data, size_t length) {
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (i2c_start(i2c->i2c_id, addr, false) != pdPASS) {
            print_uart("Error al iniciar la comunicación (SDS).\n\r");
            xSemaphoreGive(i2c->mutex);
            return pdFALSE;
        }
        for (size_t i = 0; i < length; i++) {
            if (i2c_write(i2c->i2c_id, data[i]) != pdPASS) {
                print_uart("Error al enviar datos (SDS - write).\n\r");
                i2c_send_stop(i2c->i2c_id); // Detener la comunicación
                xSemaphoreGive(i2c->mutex);
                return pdFALSE;
            }
        }
        i2c_send_stop(i2c->i2c_id);
        //i2c_clear_stop(i2c->i2c_id);
        xSemaphoreGive(i2c->mutex);
    } else {
        print_uart("Error: No se pudo obtener el mutex.\n\r");
        return pdFALSE;
    }
    
    return pdPASS;
}




/******************************
 * TESTING
 * ***************************/


/** 
 * @brief Imprimir un mensaje por I2C1 al esclavo con dirección I2C_ARDUINO_ADDRESS
 * 
 * @param data Cadena de caracteres a enviar
 * @return pdPASS si la comunicación fue exitosa, pdFALSE si hubo un error
 * 
 * @note Esta función es llamada por la tarea de testing de I2C
 */

BaseType_t print_i2c(const char *data, uint8_t addr) {
    // Inicio la comunicación -> Envío el comando -> Envío los datos -> Detengo la comunicación
    uint8_t buffer[1 + strlen(data)];
    buffer[0] = 0x01;
    memcpy(&buffer[1], data, strlen(data));
    return i2c_send_data_slave(I2C1, addr, buffer, strlen(data) + 1);
}