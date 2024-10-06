#include "i2c.h"

/**************************************** ESTRUCTURAS Y VARIABLES ****************************************/
/**
 * @brief Estructura para manejar el periférico I2C
 * 
 * @param i2c_id Identificador del periférico I2C (I2C1 o I2C2)
 * @param txq Cola de mensajes para transmitir datos
 * @param rxq Cola de mensajes para recibir datos
 * @param mutex Semáforo para controlar el acceso a los registros del periférico
 * @param request Semáforo binario para solicitar acceso al periférico
 */
typedef struct {
    uint32_t i2c_id;
    QueueHandle_t responses;
    SemaphoreHandle_t mutex;
} i2c_t;

/**
 * @brief Estructura para manejar un mensaje I2C
 * 
 * @param addr Dirección I2C del esclavo
 * @param data Datos a enviar
 * @param request true si es una solicitud
 */
typedef struct {
    uint8_t addr;  // Dirección I2C del esclavo
    uint8_t data[I2C_MAX_BUFFER];  // Datos a enviar
    size_t length;  // Longitud de los datos
    bool request;  // true si es una solicitud
} msg_t;

// Handlers de los puertos I2C
static i2c_t i2c1;
static i2c_t i2c2;

/**************************************** FUNCIONES PRIVADAS ****************************************/
/**
 * @brief Obtiene el handler de I2C basado en el identificador de 32 bits del periférico
 * 
 * @param i2c_id Identificador del periférico I2C (I2C1 o I2C2)
 * @return Puntero a la estructura que maneja el periférico I2C correspondiente o NULL si no se encuentra
 */

static i2c_t * get_i2c(uint32_t i2c_id) {
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
 * @brief Encola un mensaje en la cola especificada
 * 
 * @param msg Puntero al mensaje a encolar [REVISAR] (!)
 * @param queue Cola donde se encolará el mensaje
 * @return pdPASS si el mensaje fue encolado, pdFALSE si la cola está llena o hubo un error
 */
static BaseType_t enqueue_i2c_msg(msg_t *msg, QueueHandle_t queue) {
    if (uxQueueSpacesAvailable(queue) == 0) {
        return pdFALSE;
    }

    msg_t msg_copy;
    memcpy(&msg_copy, msg, sizeof(msg_t)); // Copiar el mensaje
    if (xQueueSend(queue, &msg_copy, pdMS_TO_TICKS(10)) != pdPASS) {
        return pdFALSE;
    }
    return pdPASS;
}


/**
 * @brief Desencola un mensaje de la cola especificada
 * 
 * @param queue Cola de mensajes
 * @return Mensaje desencolado
 */
static msg_t dequeue_i2c_msg(QueueHandle_t queue) {
    msg_t msg;
    if (xQueueReceive(queue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
        msg.addr = 0;
        return msg;
    }
    return msg;
}

/**
 * @brief Reinicia el periférico I2C especificado
 * 
 * 
 */
void reset_i2c_peripheral(uint32_t i2c_id) {
    // Deshabilitar el periférico I2C
    I2C_CR1(i2c_id) &= ~I2C_CR1_PE;
    
    // Esperar un breve momento (opcional)
    for (volatile int i = 0; i < 1000; i++);
    
    // Volver a habilitar el periférico I2C
    I2C_CR1(i2c_id) |= I2C_CR1_PE;
}


/**
 * @brief Espera hasta que el periférico I2C especificado esté listo
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return bool true si el bus está listo, false si hubo un timeout
 */

static bool i2c_wait_until_ready(uint32_t i2c_id) {
    uint32_t start_time = xTaskGetTickCount();  // Obtener el tiempo actual del sistema (FreeRTOS)

    // Espera hasta que el bus esté listo o el timeout expire
    while (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
            // Si el timeout ha sido excedido
            reset_i2c_peripheral(i2c_id);  // Reiniciar el periférico I2C
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

static bool i2c_wait_until_start(uint32_t i2c_id) {
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

static bool i2c_wait_until_address(uint32_t i2c_id) {
    TickType_t start_time = xTaskGetTickCount();  // Obtener el tiempo actual del sistema (FreeRTOS)
    TickType_t timeout = pdMS_TO_TICKS(I2C_TIMEOUT_MS);  // Establecer el tiempo de timeout

    // Espera hasta que el bit de start esté establecido o el timeout expire
    while (!(I2C_SR1(i2c_id) & I2C_SR1_ADDR)) {
        if ((xTaskGetTickCount() - start_time) > pdMS_TO_TICKS(I2C_TIMEOUT_MS)) {
            return false;  // Indicar que hubo un timeout
        }

        // Verificar si ocurrió un NACK
        if (I2C_SR1(i2c_id) & I2C_SR1_AF) {
            I2C_SR1(i2c_id) &= ~I2C_SR1_AF; // Limpiar bandera de fallo de ACK
            i2c_send_stop(i2c_id); // Detener comunicación
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
static BaseType_t i2c_start(uint32_t i2c_id, uint8_t addr, bool read) {
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
static BaseType_t i2c_write(uint32_t i2c_id, uint8_t data) {
    TickType_t start_time = xTaskGetTickCount();  // Guardamos el tiempo de inicio
    TickType_t timeout = pdMS_TO_TICKS(I2C_TIMEOUT_MS);  // Establecemos el tiempo de timeout

    i2c_send_data(i2c_id, data);

    // Esperar hasta que se complete la transferencia de datos o el timeout expire
    while (!(I2C_SR1(i2c_id) & I2C_SR1_BTF)) {
        // Verificamos si el timeout ha expirado
        if ((xTaskGetTickCount() - start_time) > timeout) {
            // Timeout: limpiar el registro I2C y devolver error
            i2c_peripheral_disable(i2c_id);  // Desactivar I2C
            i2c_peripheral_enable(i2c_id);   // Rehabilitar I2C para restablecer los registros
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
        i2c_peripheral_disable(i2c_id);
        i2c_peripheral_enable(i2c_id);
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
static uint8_t i2c_read(uint32_t i2c_id, bool last) {
    if (last) {
        i2c_disable_ack(i2c_id);
    } else {
        i2c_enable_ack(i2c_id);
    }

    while (!(I2C_SR1(i2c_id) & I2C_SR1_RxNE)) {
        // Esperar hasta que el buffer de recepción no esté vacío
        taskYIELD();
    }

    return i2c_get_data(i2c_id);
}

/**
 * @brief Verifica la conexión con un esclavo I2C en la dirección especificada
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param direccion_esclavo Dirección de 8 bits del esclavo I2C
 * @return pdPASS si la conexión fue exitosa, pdFALSE si hubo un error
 */

static BaseType_t verificar_conexion_i2c(uint32_t i2c_id, uint8_t direccion_esclavo) {
    // Genera la condición de START
    i2c_send_start(i2c_id);

    // Esperar hasta que se envíe la condición de START correctamente
    while (!(I2C_SR1(i2c_id) & I2C_SR1_SB)) {
        taskYIELD();  // Liberar la CPU mientras espera
    }

    // Enviar la dirección del esclavo con el bit de escritura (dirección << 1)
    i2c_send_7bit_address(i2c_id, direccion_esclavo, I2C_WRITE);

    // Esperar a que se envíe la dirección y verificar la respuesta del esclavo
    while (!(I2C_SR1(i2c_id) & I2C_SR1_ADDR)) {
        // Si se detecta un NACK, el esclavo no está presente
        if (I2C_SR1(i2c_id) & I2C_SR1_AF) {
            // Limpiar el bit de NACK
            I2C_SR1(i2c_id) &= ~I2C_SR1_AF;
            
            // Enviar condición de STOP
            i2c_send_stop(i2c_id);

            return pdFALSE;  // El esclavo no respondió
        }
        taskYIELD();  // Liberar la CPU mientras espera
    }

    // Se recibió un ACK del esclavo, limpiamos la bandera de dirección recibida
    (void) I2C_SR2(i2c_id);

    // Enviar condición de STOP
    i2c_send_stop(i2c_id);

    return pdPASS;  // El esclavo respondió correctamente
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
        i2c_peripheral_enable(i2c->i2c_id); // Habilitar I2C2
    } else if(i2c_id == I2C2){
        i2c -> i2c_id = I2C2;
        rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
        rcc_periph_clock_enable(RCC_I2C2); // Habilitar el reloj para I2C2

        gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                    GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C2_SCL | GPIO_I2C2_SDA); // Configurar pines para I2C2

        i2c_peripheral_disable(i2c->i2c_id); // Desactivar I2C2 antes de configurar
        rcc_periph_reset_pulse(RST_I2C2);

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
static BaseType_t i2c_make_request(uint32_t i2c_id, uint8_t addr, size_t length) {
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
        i2c_send_stop(i2c->i2c_id);
        return pdFALSE;
    }
    //taskENTER_CRITICAL(); // Iniciar sección crítica
    // Almacenar los datos recibidos en el buffer estático
    for (size_t i = 0; i < length; i++) {
        data = i2c_read(i2c->i2c_id, (i == (length - 1))); // true solo para el último byte
        if(xQueueSend(i2c->responses, &data, pdMS_TO_TICKS(10)) != pdPASS){
            print_uart("Error: No se pudo encolar el mensaje de solicitud.\n\r");
            i2c_send_stop(i2c->i2c_id);
            return pdFALSE;
        }
    }
    i2c_send_stop(i2c->i2c_id);
    //taskEXIT_CRITICAL(); // Finalizar sección crítica>
    

    return pdPASS;
}

/**
 * @brief Procedimiento para solicitar la temperatura al sensor HTU21D
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * 
 * @return pdPASS si la solicitud fue exitosa, pdFALSE si hubo un error
 */

static BaseType_t request_htu21d(uint32_t i2c_id, uint8_t command) {
    BaseType_t status;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (i2c_start(i2c->i2c_id, HTU21D_ADDRESS, false) == pdPASS) {
            i2c_write(i2c->i2c_id, command);
            i2c_send_stop(i2c->i2c_id);
        } else {
            print_uart("Error al iniciar comunicacion (comando).\n\r");
            i2c_send_stop(i2c->i2c_id);
            xSemaphoreGive(i2c->mutex);
            return pdFALSE;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Espera suficiente para la lectura

        if(i2c_make_request(i2c_id, HTU21D_ADDRESS, 3) != pdPASS){
            print_uart("Error: No se pudo realizar la solicitud.\n\r");
            xSemaphoreGive(i2c->mutex);
            return pdFALSE;
        }

        xSemaphoreGive(i2c->mutex);
        
    } else {
        print_uart("Error: No se pudo obtener el mutex.\n\r");
        return pdFALSE;
    }

    return pdPASS;
}

static BaseType_t reset_htu21d(uint32_t i2c_id){
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    for (int retry_count = 0; retry_count < 3; retry_count++) {
        if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (i2c_start(i2c->i2c_id, HTU21D_ADDRESS, false) == pdPASS) {
                i2c_write(i2c->i2c_id, SOFT_RESET);
                i2c_send_stop(i2c->i2c_id);
                vTaskDelay(pdMS_TO_TICKS(15));
                xSemaphoreGive(i2c->mutex);
                vTaskDelay(pdMS_TO_TICKS(15));
                return pdTRUE;  // Comunicación exitosa
            } else {
                i2c_send_stop(i2c->i2c_id);
                xSemaphoreGive(i2c->mutex);
                vTaskDelay(pdMS_TO_TICKS(100));  // Espera antes del siguiente intento
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));  // Espera antes del siguiente intento
        }
    }
   
    return pdFALSE;

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
static BaseType_t i2c_send_data_slave(uint32_t i2c_id, uint8_t addr, uint8_t* data, size_t length) {
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (i2c_start(i2c->i2c_id, addr, false) != pdPASS) {
            print_uart("Error al iniciar la comunicación (SDS).\n\r");
            i2c_send_stop(i2c->i2c_id); // Detener la comunicación
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


/**
 * @brief Resetea el bus I2C especificado
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return void
 */
static void i2c_reset_bus(uint32_t i2c_id) {
    // Apagar el periférico I2C
    i2c_peripheral_disable(i2c_id);
    
    // Breve espera
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    // Reiniciar el periférico I2C
    i2c_peripheral_enable(i2c_id);
}



/******************************
 * TESTING
 * ***************************/

/**
 * @brief Imprime un mensaje por UART
 * 
 * @param s Mensaje a imprimir
 * @return void
 */
void print_uart(const char *s){
    UART_puts(USART1, s, pdMS_TO_TICKS(500));
}




/**
 * @brief Tarea de testing para solicitar la temperatura y ebviarla al Arduino
 *       Solicita tres bytes al HTU21D, calcula la temperatura y la envía al Arduino, todo por I2C
 * 
 * @param pvParameters Sin utilizar
 * @return void
 */

void test_request_i2c(void *pvParameters) {
    uint32_t i2c_id = I2C1;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C. Eliminando tarea...\n\r");
        vTaskDelete(NULL);
    }
    
    bool reset = false;

    for (;;) {
        uint8_t data[3];
        char data_str[20]; // Asegúrate de que el tamaño sea suficiente para la cadena
        /*
        if(reset){
            i2c_reset_bus(i2c_id);
            print_uart("Bus I2C reiniciado.\n\r");
            reset = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        */
        if(reset_htu21d(i2c_id) != pdPASS){
            print_uart("Error: No se pudo realizar el reset del sensor.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (request_htu21d(i2c_id, TRIGGER_TEMP_MEASURE_NOHOLD) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de temperatura.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));

        // Desencolo response
        uint8_t data_aux;
        bool error = false;

        for(int i = 0; i < 3; i++){
            if(xQueueReceive(i2c->responses, &data_aux, pdMS_TO_TICKS(100)) != pdTRUE){
                print_uart("Error: No se pudo recibir el mensaje de temperatura.");
                error = true;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            data[i] = data_aux;
        }

        if(error){
            reset = true;
            continue;
        }

        uint16_t rawTemperature = (data[0] << 8) | data[1];
        rawTemperature &= 0xFFFC;

        float temp = -46.85 + (175.72 * rawTemperature / 65536.0);  // Convertimos los datos a temperatura en grados Celsius
   
        snprintf(data_str, sizeof(data_str), "Temp: %.2f °C", temp); // Formatea la temperatura a dos decimales como cadena
    
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != pdPASS){
            print_uart("Error: No se pudo enviar el mensaje data_str.\r\n");
            reset = true;
            continue;
        }
        

        vTaskDelay(pdMS_TO_TICKS(2500));
        memset(data_str, 0, sizeof(data_str));

        if (request_htu21d(i2c_id, TRIGGER_HUMD_MEASURE_NOHOLD) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de humedad.");
            reset = true;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));

        // Desencolo response
        for(int i = 0; i < 3; i++){
            if(xQueueReceive(i2c->responses, &data_aux, pdMS_TO_TICKS(100)) != pdTRUE){
                print_uart("Error: No se pudo recibir el mensaje de humedad.");
                error = true;
                break;
            }
            data[i] = data_aux;
        }

        if(error){
            reset = true;
            continue;
        }

        uint16_t rawHumidity = (data[0] << 8) | data[1];  // Leemos los dos primeros bytes

        rawHumidity &= 0xFFFC;  // Aplicamos máscara para quitar los bits de estado
        float hum = -6.0 + (125.0 * rawHumidity / 65536.0);  // Convertimos los datos a porcentaje de humedad

        snprintf(data_str, sizeof(data_str), "Hum: %.2f %%", hum); // Formatea la temperatura a dos decimales como cadena
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != pdPASS){
            print_uart("Error: No se pudo enviar el mensaje hum_str.");
            reset = true;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));
        print_uart("---------------\n\r");
    }
}

/**
 * @brief Tarea de testing que envia un mensaje periódico al 0x08 por I2C, testeando el bus
 * 
 * @param pvParameters Sin utilizar
 * @return void
 */

void test_i2c(void *pvParameters) {
    uint32_t i2c_id = I2C1;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C. Eliminando tarea...\n\r");
        vTaskDelete(NULL);
    }

    for (;;) {
        if (print_i2c("Hola, Arduino!", 0x08) != pdPASS) {
            print_uart("Error: No se pudo enviar el mensaje (UNO).\n\r");
            xSemaphoreGive(i2c->mutex);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }


        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}




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