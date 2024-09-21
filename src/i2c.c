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
    QueueHandle_t txq;
    QueueHandle_t rxq;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t request;
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
 * @brief Espera hasta que el periférico I2C especificado esté listo
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return void
 */

static void i2c_wait_until_ready(uint32_t i2c_id) {
    while (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
        taskYIELD();
    }
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
    i2c_wait_until_ready(i2c_id);
    i2c_send_start(i2c_id);

    // Esperar hasta que el bit de Start esté establecido
    while (!(I2C_SR1(i2c_id) & I2C_SR1_SB)) {
        taskYIELD();
    }

    i2c_send_7bit_address(i2c_id, addr, read ? I2C_READ : I2C_WRITE);

    // Esperar hasta que el bit de Address esté establecido
    while (!(I2C_SR1(i2c_id) & I2C_SR1_ADDR)) {
        // Verificar si ocurrió un NACK
        if (I2C_SR1(i2c_id) & I2C_SR1_AF) {
            I2C_SR1(i2c_id) &= ~I2C_SR1_AF; // Limpiar bandera de fallo de ACK
            i2c_send_stop(i2c_id); // Detener comunicación
            return pdFALSE; // Indicar que la comunicación falló
        }
        taskYIELD();
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
    i2c_send_data(i2c_id, data);
    // Esperar hasta que se complete la transferencia de datos
    while (!(I2C_SR1(i2c_id) & I2C_SR1_BTF)) {
        taskYIELD();
    }

    // Comprobar el bit de ACK después de la transferencia
    if (I2C_SR1(i2c_id) & I2C_SR1_AF) {
        // No se recibió ACK del esclavo (NACK)
        // Limpia la bandera de fallo de ACK
        I2C_SR1(i2c_id) &= ~I2C_SR1_AF;
        // Manejar el error de NACK aquí (por ejemplo, reenviar el mensaje o detener la comunicación)
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

    if ((i2c -> request = xSemaphoreCreateBinary()) == NULL)
        return pdFALSE;

    if((i2c -> txq = xQueueCreate(10, sizeof(msg_t))) == NULL)
        return pdFALSE;

    if((i2c -> rxq = xQueueCreate(10, sizeof(msg_t))) == NULL)
        return pdFALSE;
    
    return pdPASS;
}

/**
 * @brief Tarea para transmitir bytes por I2C, ya sean datos o solicitudes de lectura. 
 * 
 *        Espera a que haya datos en la cola de transmisión y los encola en la cola de transmisión
 * 
 * @param pvParameters Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return void
 */
void task_i2c_tx(void *pvParameters) {
    i2c_t *i2c = get_i2c((uint32_t) pvParameters);
    if (i2c == NULL || i2c->txq == NULL || i2c->rxq == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        vTaskDelete(NULL);
        return;
    }

    msg_t msg;
    const uint8_t max_retries = 10;

    for (;;) {
        // Verificar si hay datos en la cola con un tiempo de espera corto
        if (xQueueReceive(i2c->txq, &msg, pdMS_TO_TICKS(10)) == pdPASS) {
            uint8_t retries = 0;
            bool success = false;

            while (retries < max_retries && !success) {
                if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if (msg.request) {
                    // SECCIÓN REQUEST
                        // Intentar iniciar la comunicación I2C
                        if (i2c_start(i2c->i2c_id, msg.addr, true)) {
                            //taskENTER_CRITICAL(); // Iniciar sección crítica
                            // Almacenar los datos recibidos en el buffer estático
                            for (size_t i = 0; i < msg.length; i++) {
                                msg.data[i] = i2c_read(i2c->i2c_id, (i == (msg.length - 1))); // true solo para el último byte
                            }
                            i2c_send_stop(i2c->i2c_id);
                            enqueue_i2c_msg(&msg, i2c->rxq); // Encolar el dato recibido
                            success = true;
                            //taskEXIT_CRITICAL(); // Finalizar sección crítica
                        }
                    } else {
                    // SECCIÓN WRITE
                        // Intentar iniciar la comunicación I2C
                        if (i2c_start(i2c->i2c_id, msg.addr, false)) {
                            for (size_t i = 0; i < msg.length; i++) {
                                i2c_write(i2c->i2c_id, msg.data[i]); // Enviar byte por byte
                            }
                            i2c_send_stop(i2c->i2c_id);
                            success = true;
                        } else {
                            print_uart("Error: No se pudo establecer la comunicación I2C (TX).\n\r");
                        }
                    }
                    xSemaphoreGive(i2c->mutex);
                } else {
                    print_uart("Error: No se pudo obtener el mutex.\n\r");
                }
                retries++;
            }

            if (!success) {
                print_uart("Error: No se pudo enviar el mensaje.\n\r");
                vTaskDelay(pdMS_TO_TICKS(3000)); // Esperar 3 segundos antes de reintentar
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10)); // Para reducir la carga del CPU
        }

        // Ceder la ejecución para permitir que otras tareas se ejecuten
        taskYIELD();
    }
}


/**
 * @brief Tarea para leer bytes por I2C y mostrarlos por UART
 *        [REVISAR: generalizar para cualquier uso] (!)
 * 
 * @param pvParameters Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return void
 */
void task_read_i2c(void *pvParameters) {
    i2c_t *i2c = get_i2c((uint32_t) pvParameters);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        vTaskDelete(NULL);
    }

    for (;;) {
        // Verificar si hay mensajes en la cola de recepción
        if (uxQueueMessagesWaiting(i2c->rxq) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Esperar 10 ms antes de la próxima verificación
        } else {
            msg_t msg = dequeue_i2c_msg(i2c->rxq);
            print_uart("Datos recibidos: ");

            // Agregar todos los bytes del buffer de datos a la cadena
            for (size_t i = 0; i < msg.length; i++) {
                // Paso el byte a char para imprimirlo
                char c[3];
                sprintf(c, "%02X", msg.data[i]);
                print_uart(c);
                print_uart(" ");
            }
            // Agregar un salto de línea al final
            print_uart("\n\r");
        }
    }
}



/**
 * @brief Función para solicitar datos por I2C
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param slave_addr Dirección de 8 bits del esclavo I2C
 * @param length Longitud de los datos a solicitar
 * @return pdPASS si la solicitud fue realizada, pdFALSE si hubo un error
 */
BaseType_t i2c_request_from(uint32_t i2c_id, uint8_t slave_addr, size_t length) {
    if (length == 0 || length > I2C_MAX_BUFFER) {
        print_uart("Error: Longitud inválida.\n\r");
        return pdFALSE;
    }

    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    // Crear el mensaje de solicitud
    msg_t msg;
    msg.addr = slave_addr;
    msg.length = length;
    msg.request = true;

    // Encolar el mensaje en la cola de transmisión
    if (enqueue_i2c_msg(&msg, i2c->txq) != pdPASS) {
        print_uart("Error: No se pudo encolar el mensaje de solicitud.\n\r");
        return pdFALSE;
    }

    return pdPASS;
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
 * @brief Tarea para escribir bytes por I2C1 en modo WRITE
 *        Envia un conteo creciente a partir de 0
 * 
 * @param pvParameters Dirección I2C del esclavo
 * @return void
 */
void test_write_i2c(void *pvParameters) {
    uint32_t i2c_id = I2C1;
    uint8_t slave_addr = (uint8_t) pvParameters;
    uint8_t data[I2C_MAX_BUFFER]; // Arreglo para almacenar datos a enviar
    size_t length = sizeof(data); // Longitud del arreglo

    for (size_t i = 0; i < length; i++) {
        data[i] = i; // Asignar valores crecientes
    }

    for (;;) {
        if (write_data(i2c_id, slave_addr, data, length) == pdFALSE) {
            print_uart("Error: No se pudo enviar el mensaje.\n\r");
        }

        vTaskDelay(pdMS_TO_TICKS(2500)); // Esperar 2.5 segundos antes de la próxima solicitud
    }
}


/**
 * @brief Tarea para solicitar bytes por I2C1 en modo READ
 *       Solicita un byte al esclavo y lo imprime por UART
 * 
 * @param pvParameters Dirección I2C del esclavo
 * @return void
 */

void test_request_i2c(void *pvParameters) {

    for (;;) {
        if (i2c_request_from(I2C1, 0x04, 5) == pdFALSE) {
            print_uart("Error: No se pudo realizar la solicitud.\n\r");
        }

        vTaskDelay(pdMS_TO_TICKS(2500)); // Esperar 2.5 segundos antes de la próxima solicitud
    }
}
