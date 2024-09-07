#include "i2c.h"

SemaphoreHandle_t i2c_mutex;
QueueHandle_t i2c_tx_queue;
QueueHandle_t i2c_rx_queue;
int mensaje = 0;

void i2c_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
    rcc_periph_clock_enable(RCC_I2C1); // Habilitar el reloj para I2C1

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA); // Configurar pines para I2C

    i2c_peripheral_disable(I2C1); // Desactivar I2C1 antes de configurar
    rcc_periph_reset_pulse(RST_I2C1);

    i2c_set_standard_mode(I2C1); // Configurar modo estándar
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ); // Configurar frecuencia del reloj a 36 MHz
    i2c_set_trise(I2C1, 36); // Configurar tiempo de subida
    i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2); // Configurar ciclo de trabajo
    i2c_set_ccr(I2C1, 180); // Configurar CCR para 100 kHz
    i2c_peripheral_enable(I2C1); // Habilitar I2C1

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        print_uart("Error al crear el semáforo\n\r");
        while(1);
    }

    i2c_tx_queue = xQueueCreate(10, sizeof(uint8_t)); // Cola para datos a enviar
    i2c_rx_queue = xQueueCreate(10, sizeof(uint8_t)); // Cola para datos recibidos
}

void i2c_wait_until_ready(void) {
    while (I2C_SR2(I2C1) & I2C_SR2_BUSY) {
        taskYIELD();
    }
}

bool i2c_start(uint8_t addr, bool read) {
    i2c_wait_until_ready();
    i2c_send_start(I2C1);

    // Esperar hasta que el bit de Start esté establecido
    while (!(I2C_SR1(I2C1) & I2C_SR1_SB)) {
        taskYIELD();
    }

    i2c_send_7bit_address(I2C1, addr, read ? I2C_READ : I2C_WRITE);

    // Esperar hasta que el bit de Address esté establecido
    while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR)) {
        // Verificar si ocurrió un NACK
        if (I2C_SR1(I2C1) & I2C_SR1_AF) {
            I2C_SR1(I2C1) &= ~I2C_SR1_AF; // Limpiar bandera de fallo de ACK
            print_uart("Error al establecer la comunicacion\n\r");
            i2c_send_stop(I2C1); // Detener comunicación
            return false; // Indicar que la comunicación falló
        }
        taskYIELD();
    }

    // Limpiar la bandera de dirección
    (void)I2C_SR2(I2C1);

    return true; // Indicar que la comunicación fue exitosa
}



void i2c_write(uint8_t data) {
    i2c_send_data(I2C1, data);
    // Esperar hasta que se complete la transferencia de datos
    while (!(I2C_SR1(I2C1) & I2C_SR1_BTF)) {
        taskYIELD();
    }

    // Comprobar el bit de ACK después de la transferencia
    if (I2C_SR1(I2C1) & I2C_SR1_AF) {
        // No se recibió ACK del esclavo (NACK)
        // Limpia la bandera de fallo de ACK
        I2C_SR1(I2C1) &= ~I2C_SR1_AF;
        // Manejar el error de NACK aquí (por ejemplo, reenviar el mensaje o detener la comunicación)
        print_uart("No se recibio ACK\n\r");
    } else {
        // ACK recibido correctamente
        
    }
}


uint8_t i2c_read(bool last) {
    if (last) {
        i2c_disable_ack(I2C1);
    } else {
        i2c_enable_ack(I2C1);
    }

    while (!(I2C_SR1(I2C1) & I2C_SR1_RxNE)) {
        // Esperar hasta que el buffer de recepción no esté vacío
        taskYIELD();
    }

    return i2c_get_data(I2C1);
}

void i2c_stop(void) {
    i2c_send_stop(I2C1);

}

void enqueue_i2c_data(uint8_t data, QueueHandle_t queue) {
    
    // Verificar si la cola está llena
    if (uxQueueSpacesAvailable(queue) == 0) {
        print_uart("La cola I2C está llena\n\r");
        return; // Opcionalmente, puedes retornar o manejar el error de otra manera
    }

    // Intentar enviar el dato a la cola
    if (xQueueSend(queue, &data, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        print_uart("Error al encolar datos I2C\n\r");
    }
}


uint8_t dequeue_i2c_data(QueueHandle_t queue) {
    uint8_t data;
    if (xQueueReceive(queue, &data, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        print_uart("Error al desencolar datos I2C\n\r");
    }
    return data;
}

void task_i2c_tx(void *pvParameters) {
    uint8_t data;
    for (;;) {
        // Verificar si hay datos en la cola con un tiempo de espera corto
        if (xQueueReceive(i2c_tx_queue, &data, pdMS_TO_TICKS(10)) == pdPASS) {
            // Esperar a obtener el mutex
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                // Intentar iniciar la comunicación I2C
                if (i2c_start(I2C_SLAVE_ADDRESS, false)) {
                    i2c_write(data);
                    i2c_stop();
                } else {
                    // La comunicación no se pudo establecer
                    print_uart("Error: No se pudo establecer la comunicación I2C.\n\r");
                    // Dependiendo del caso, podrías manejar el error aquí, como reintentar, notificar, etc.
                }

                // Liberar el mutex después de completar la transmisión o manejo de error
                xSemaphoreGive(i2c_mutex);
            } else {
                print_uart("Error: No se pudo obtener el mutex.\n\r");
            }
        }

        // Ceder la ejecución para permitir que otras tareas se ejecuten
        taskYIELD();
    }
}

void task_i2c_rx(void *pvParameters) {
    uint8_t data;
    for (;;) {
        // Esperar a obtener el mutex
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            i2c_start(I2C_SLAVE_ADDRESS, true);
            data = i2c_read(true);
            i2c_stop();
            xSemaphoreGive(i2c_mutex); // Liberar el mutex
            enqueue_i2c_data(data, i2c_rx_queue); // Encolar el dato recibido
        }

        taskYIELD();
    }
}

// Tarea para enviar y recibir datos a través de I2C

void task_i2c(void *pvParameters) {
    uint8_t respuesta;
    char buffer[50]; // Ajusta el tamaño según sea necesario
    
    for(;;) {
        if (uxQueueSpacesAvailable(i2c_tx_queue) > 0) {
            enqueue_i2c_data(mensaje, i2c_tx_queue); // Encolar mensaje en cola de transmisión
        } else {
            print_uart("Espacio insuficiente\n\r");
        }

        //vTaskDelay(pdMS_TO_TICKS(100)); // Esperar a que el Arduino procese el comando

        //i2c_start(I2C_SLAVE_ADDRESS, true); // Leer desde el Arduino
        //respuesta = i2c_read(true); // Leer la hora (último byte)
        //i2c_stop();
        mensaje++;
        vTaskDelay(pdMS_TO_TICKS(5000)); // Esperar 1 segundo antes de la próxima solicitud
    }
}











/******************************
 * TESTING
 * ***************************/
 
 void print_uart(const char *s){
    UART_puts(USART1, s, pdMS_TO_TICKS(500));
}