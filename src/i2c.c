#include "i2c.h"

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
    uint8_t data;  // Datos a enviar
    bool request;  // true si es una solicitud
} msg_t;

static i2c_t i2c1;
static i2c_t i2c2;

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
    if (uxQueueSpacesAvailable(queue) == 0)
        return pdFALSE;

    if (xQueueSend(queue, msg, pdMS_TO_TICKS(10)) != pdPASS) {
        pdFALSE;
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
        msg.data = 0;
        return msg;
    }
    return msg;
}


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
 * @brief Espera hasta que el periférico I2C especificado esté listo
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return void
 */
void i2c_wait_until_ready(uint32_t i2c_id) {
    while (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
        taskYIELD();
    }
}

/************************ PROCESO DE INICIO DE COMUNICACIÓN I2C ************************/
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


/************************ PROCESO DE TRANSMISION DE UN MENSAJE ************************/
// 1. Encolar un msg_t con la dirección y el dato a enviar en TXQ. El flag request debe estar en false
// 2. Iniciar la comunicación I2C en modo WRITE con i2c_start (puerto I2C, dirección del esclavo, false)
// 3. Enviar el byte de datos con i2c_write (puerto I2C, byte de datos)
// 4. Esperar a que se complete la transferencia de datos
// 5. Comprobar si se recibió ACK del esclavo
// 6. Enviar un STOP para finalizar la comunicación

/*                  PROCESO DE SOLICITUD Y RECEPCIÓN DE UN MENSAJE                  */
// 1. Encolar un msg_t con la dirección del esclavo en TXQ. El flag request debe estar en true
// 2. Iniciar la comunicación I2C en modo READ con i2c_start (puerto I2C, dirección del esclavo, true)
// 3. Leer el byte de datos con i2c_read (puerto I2C, true) y encolarlo en RXQ
// 4. Enviar un NACK si es el último byte a leer
// 5. Enviar un STOP para finalizar la comunicación


/**
 * @brief Envia un byte de datos por I2C en modo WRITE
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param data Byte de datos a enviar
 * @return pdPASS si el byte fue enviado, pdFALSE si hubo un error
 */
BaseType_t i2c_write(uint32_t i2c_id, uint8_t data) {
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
uint8_t i2c_read(uint32_t i2c_id, bool last) {
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
 * @brief Tarea para transmitir bytes por I2C, ya sean datos o solicitudes de lectura. 
 * 
 *        Espera a que haya datos en la cola de transmisión y los encola en la cola de transmisión
 * 
 * @param pvParameters Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return void
 */
void task_i2c_tx(void *pvParameters) {
    i2c_t * i2c = get_i2c((uint32_t) pvParameters);
    if(i2c == NULL || i2c -> txq == NULL || i2c -> rxq == NULL){
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        vTaskDelete(NULL);
        return;
    }
    msg_t msg;
    const uint8_t max_retries = 10;
    for (;;) {
        // Verificar si hay datos en la cola con un tiempo de espera corto
        if (xQueueReceive(i2c -> txq, &msg, pdMS_TO_TICKS(10)) == pdPASS) {
            uint8_t retries = 0;
            bool success = false;
            while(retries < max_retries && !success){
                if (xSemaphoreTake(i2c -> mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    if(msg.request){
                        // Intentar iniciar la comunicación I2C
                        if (i2c_start(i2c->i2c_id, msg.addr, true)) {
                            msg.data = i2c_read(i2c -> i2c_id, true);
                            i2c_send_stop(i2c -> i2c_id);
                            enqueue_i2c_msg(&msg, i2c -> rxq); // Encolar el dato recibido
                            success = true;
                        } else {
                            // La comunicación no se pudo establecer
                            print_uart("Error: No se pudo establecer la comunicación I2C (RQT).\n\r");
                        }
                    } else {
                        // Intentar iniciar la comunicación I2C
                        if (i2c_start(i2c->i2c_id, msg.addr, false)) {
                            i2c_write(i2c->i2c_id, msg.data);
                            i2c_send_stop(i2c->i2c_id);
                            success = true;
                        } else {
                            // La comunicación no se pudo establecer
                            print_uart("Error: No se pudo establecer la comunicación I2C (TX).\n\r");
                        }                    
                    }
                    xSemaphoreGive(i2c -> mutex);
                } else {
                    print_uart("Error: No se pudo obtener el mutex.\n\r");
                }
                retries++;
            }

            if(!success){
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
    i2c_t * i2c = get_i2c((uint32_t) pvParameters);
    if(i2c == NULL){
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        vTaskDelete(NULL);
    }
    for(;;) {
        // Imprimo el dato recibido
        if(uxQueueMessagesWaiting(i2c -> rxq) == 0){
            vTaskDelay(pdMS_TO_TICKS(10)); // Esperar 1 segundo antes de la próxima solicitud
        } else {
            msg_t msg = dequeue_i2c_msg(i2c -> rxq);
            char buffer[50];
            snprintf(buffer, 50, "Dato recibido: %d\n\r", msg.data);
            print_uart(buffer);
        }
    }
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
    i2c_t * i2c = get_i2c(I2C1);
    msg_t msg;
    msg.addr = (uint8_t) pvParameters;
    msg.request = false;
    msg.data = 0;
    
    for(;;) {
        if (uxQueueSpacesAvailable(i2c -> txq) > 0) {
            enqueue_i2c_msg(&msg, i2c -> txq); // Encolar mensaje en cola de transmisión
        } else {
            print_uart("Espacio insuficiente\n\r");
        }

        msg.data++;
        vTaskDelay(pdMS_TO_TICKS(2500)); // Esperar 5 segundos antes de la próxima solicitud
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
    i2c_t * i2c = get_i2c(I2C1);
    msg_t msg;
    msg.addr = (uint8_t) pvParameters;
    msg.request = true;
    msg.data = 0;
    
    for(;;) {
        if (uxQueueSpacesAvailable(i2c -> txq) > 0) {
            enqueue_i2c_msg(&msg, i2c -> txq); // Encolar mensaje en cola de transmisión
        } else {
            print_uart("Espacio insuficiente\n\r");
        }

        vTaskDelay(pdMS_TO_TICKS(2500)); // Esperar 5 segundos antes de la próxima solicitud
    }
}
