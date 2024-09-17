#include "i2c.h"

typedef struct {
    uint32_t i2c_id;
    QueueHandle_t txq;
    QueueHandle_t rxq;
    SemaphoreHandle_t mutex;
    SemaphoreHandle_t request;
} i2c_t;

typedef struct {
    uint8_t addr;  // Dirección I2C del esclavo
    uint8_t data;  // Datos a enviar
} msg_t;

static i2c_t i2c1;
static i2c_t i2c2;

static uint8_t slave_address_request = 0;

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

static void enqueue_i2c_msg(msg_t *msg, QueueHandle_t queue) {
    // Verificar si la cola está llena
    if (uxQueueSpacesAvailable(queue) == 0) {
        print_uart("La cola I2C está llena\n\r");
        return; // Opcionalmente, puedes retornar o manejar el error de otra manera
    }

    // Intentar enviar el dato a la cola
    if (xQueueSend(queue, msg, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        print_uart("Error al encolar datos I2C\n\r");
    }
}

static msg_t dequeue_i2c_msg(QueueHandle_t queue) {
    msg_t msg;
    if (xQueueReceive(queue, &msg, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        print_uart("Error al desencolar datos I2C\n\r");
    }
    return msg;
}

void i2c_setup(uint32_t i2c_id) {
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
    
    i2c -> mutex = xSemaphoreCreateMutex();
    if (i2c->mutex == NULL) {
        print_uart("Error al crear el semáforo\n\r");
        while(1);
    }

    i2c -> txq = xQueueCreate(10, sizeof(msg_t)); // Cola para datos a enviar
    i2c -> rxq = xQueueCreate(10, sizeof(msg_t)); // Cola para datos recibidos
    i2c -> request = xSemaphoreCreateBinary(); // Semáforo para solicitudes
}

void i2c_wait_until_ready(uint32_t i2c_id) {
    while (I2C_SR2(i2c_id) & I2C_SR2_BUSY) {
        taskYIELD();
    }
}

bool i2c_start(uint32_t i2c_id, uint8_t addr, bool read) {
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
            print_uart("Error al establecer la comunicacion\n\r");
            i2c_send_stop(i2c_id); // Detener comunicación
            return false; // Indicar que la comunicación falló
        }
        taskYIELD();
    }

    // Limpiar la bandera de dirección
    (void)I2C_SR2(i2c_id);

    return true; // Indicar que la comunicación fue exitosa
}

void i2c_write(uint32_t i2c_id, uint8_t data) {
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
        print_uart("No se recibio ACK\n\r");
    }
}

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



void task_i2c_tx(void *pvParameters) {
    i2c_t * i2c = get_i2c((uint32_t) pvParameters);
    msg_t msg;
    const uint8_t max_retries = 10;
    for (;;) {
        // Verificar si hay datos en la cola con un tiempo de espera corto
        if (xQueueReceive(i2c -> txq, &msg, pdMS_TO_TICKS(10)) == pdPASS) {
            uint8_t retries = 0;
            bool success = false;
            while(retries < max_retries && !success){
                if (xSemaphoreTake(i2c -> mutex, portMAX_DELAY) == pdTRUE) {
                    // Intentar iniciar la comunicación I2C
                    if (i2c_start(i2c->i2c_id, msg.addr, false)) {
                        i2c_write(i2c->i2c_id, msg.data);
                        i2c_send_stop(i2c->i2c_id);
                        success = true;
                    } else {
                        // La comunicación no se pudo establecer
                        print_uart("Error: No se pudo establecer la comunicación I2C.\n\r");
                    }                    
                } else {
                    print_uart("Error: No se pudo obtener el mutex.\n\r");
                }
                retries++;
                xSemaphoreGive(i2c -> mutex);
            }

            if(!success){
                print_uart("Error: No se pudo enviar el mensaje.\n\r");
                vTaskDelay(pdMS_TO_TICKS(3000)); // Esperar 3 segundos antes de reintentar
            }
        }

        // Ceder la ejecución para permitir que otras tareas se ejecuten
        taskYIELD();
    }
}



void task_i2c_request(void *pvParameters) {
    i2c_t * i2c = get_i2c((uint32_t) pvParameters);
    msg_t msg;
    const uint8_t max_retries = 10;
    for (;;) {
        if (xSemaphoreTake(i2c -> request, portMAX_DELAY) == pdTRUE){
            msg.addr = slave_address_request;
            uint8_t retries = 0;
            bool success = false;
            while(retries < max_retries && !success){
                if (xSemaphoreTake(i2c -> mutex, portMAX_DELAY) == pdTRUE) {
                    if(i2c_start(i2c -> i2c_id, msg.addr, true)){
                        msg.data = i2c_read(i2c -> i2c_id, true);
                        i2c_send_stop(i2c -> i2c_id);
                        xSemaphoreGive(i2c -> mutex); // Liberar el mutex
                        enqueue_i2c_msg(&msg, i2c -> rxq); // Encolar el dato recibido
                        success = true;
                    } else {
                        print_uart("Error: No se pudo establecer la comunicación I2C.\n\r");
                        xSemaphoreGive(i2c -> mutex); // Liberar el mutex
                        retries++;
                    }
                }
                if(!success){
                    print_uart("Error: No se pudo enviar el mensaje.\n\r");
                    vTaskDelay(pdMS_TO_TICKS(3000)); // Esperar 3 segundos antes de reintentar
                }
            }
        }

        taskYIELD();
    }
}


void test_write_i2c(void *pvParameters) {
    i2c_t * i2c = get_i2c(I2C1);
    msg_t msg;
    msg.addr = (uint8_t) pvParameters;
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


void test_request_i2c(void *pvParameters) {
    i2c_t * i2c = get_i2c(I2C1);

    for(;;) {
        slave_address_request = (uint8_t) pvParameters;
        xSemaphoreGive(i2c -> request);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de la próxima solicitud
        // Imprimo el dato recibido
        i2c_t * i2c = get_i2c(I2C1);
        if(uxQueueMessagesWaiting(i2c -> rxq) == 0){
            print_uart("No hay datos en la cola\n\r");
        } else {
            msg_t msg = dequeue_i2c_msg(i2c -> rxq);
            char buffer[50];
            snprintf(buffer, 50, "Dato recibido: %d\n\r", msg.data);
            print_uart(buffer);
        }
    }
}

void htu21d_send_command(uint32_t i2c_id, uint8_t command) {
    i2c_t * i2c = get_i2c(i2c_id);
    if (xSemaphoreTake(i2c -> mutex , portMAX_DELAY) == pdTRUE) {
        if (i2c_start(i2c_id, I2C_HUMIDITY_ADDRESS, false)) {
            i2c_write(i2c_id, command);
            i2c_send_stop(i2c_id);
        }
        xSemaphoreGive(i2c -> mutex);
    }
}

uint16_t htu21d_read_data(uint32_t i2c_id) {
    uint16_t data = 0;
    i2c_t * i2c = get_i2c(i2c_id);
    if (xSemaphoreTake(i2c -> mutex, portMAX_DELAY) == pdTRUE) {
        if (i2c_start(i2c_id, I2C_HUMIDITY_ADDRESS, true)) {
            data = i2c_read(i2c_id, true); // Leer un byte
            data <<= 8; // Mover el byte a la posición alta
            data |= i2c_read(i2c_id, true); // Leer el segundo byte
            i2c_send_stop(i2c_id);
        }
        xSemaphoreGive(i2c -> mutex);
    }

    return data;
}

void task_read_temperature(void *pvParameters) {
    for (;;) {
        htu21d_send_command(I2C1, 0xE3);

        // Esperar la conversión (500 ms típico)
        vTaskDelay(pdMS_TO_TICKS(500));

        uint16_t temp_raw = htu21d_read_data(I2C1);
        // Convertir los datos a grados Celsius
        float temperature = -46.85 + 175.72 * temp_raw / 65536.0;
        // paso a entero
        int temp = (int) temperature;
        
        i2c_t * i2c = get_i2c(I2C1);
        msg_t msg;
        msg.addr = (uint8_t) I2C_ARDUINO_ADDRESS;
        msg.data = (uint8_t)(temp & 0xFF);   
        if (uxQueueSpacesAvailable(I2C1) > 0) {
            enqueue_i2c_msg(&msg, i2c -> txq); // Encolar mensaje en cola de transmisión
        } else {
            print_uart("Espacio insuficiente\n\r");
        }

        msg.data = (uint8_t)((temp >> 8) & 0xFF); // Byte más significativo
        if (uxQueueSpacesAvailable(i2c -> txq) > 0) {
            xQueueSend(i2c -> txq, &msg, portMAX_DELAY); // Enviar el segundo byte
        } else {
            print_uart("Espacio insuficiente\n\r");
        }


        vTaskDelay(pdMS_TO_TICKS(2000)); // Esperar 2 segundos antes de la próxima lectura
    }
}

/*
void task_read_humidity(void *pvParameters) {
    for (;;) {
        htu21d_send_command(I2C1, 0xE5);

        // Esperar la conversión (500 ms típico)
        vTaskDelay(pdMS_TO_TICKS(500));

        uint16_t hum_raw = htu21d_read_data(I2C1);
        // Convertir los datos a porcentaje de humedad
        float humidity = -6.0 + 125.0 * hum_raw / 65536.0;
        char buffer[50];
        snprintf(buffer, 50, "Humidity: %.2f %%\n\r", humidity);
        print_uart(buffer);

        vTaskDelay(pdMS_TO_TICKS(2000)); // Esperar 2 segundos antes de la próxima lectura
    }
}
*/



/******************************
 * TESTING
 * ***************************/
 
void print_uart(const char *s){
    UART_puts(USART1, s, pdMS_TO_TICKS(500));
}



