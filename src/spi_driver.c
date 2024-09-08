#include "FreeRTOS.h"
#include "spi_driver.h"
#include <libopencm3/stm32/rcc.h>
#include <string.h>

QueueHandle_t SPI1_rxq; //Creo la cola de recepcion
QueueHandle_t SPI1_txq; //Creo la cola de recepcion
QueueHandle_t SPI2_rxq; //Creo la cola de recepcion
QueueHandle_t SPI2_txq; //Creo la cola de recepcion

#define SPI_SIZE_BUFFER 256  // Queues size

void spi_setup(void) {
    SPI1_rxq = xQueueCreate(SPI_SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de recepción
    SPI1_txq = xQueueCreate(SPI_SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de transmisión

    rcc_periph_clock_enable(RCC_SPI1); //Enable the clock for SPI1
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO4 | GPIO5 | GPIO7 // NSS=PA4, SCK=PA5, MOSI=PA7
    );

    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO6 // MISO=PA6
    );

    //spi_reset(SPI1);
    
    spi_init_master(
        SPI1,
        SPI_CR1_BAUDRATE_FPCLK_DIV_256,     //Se debe tener en cuenta la maxima frecuencia de operacion de APB1 y APB2
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    //SCK en estado bajo en el estado inactivo (IDLE)
        SPI_CR1_CPHA_CLK_TRANSITION_1,      //Determina la fase del CLK (en que flanco se captura)
        SPI_CR1_DFF_8BIT,                   //Indica el largo de palabra a utilizar
        SPI_CR1_MSBFIRST                    //Se transmite el MSB primero
    );

    spi_disable_software_slave_management(SPI1);    //Desactivo el manejo del NSS por software
    spi_enable_ss_output(SPI1);    //Activo el manejo del NSS por hardware (SPI PHP)
    spi_enable(SPI1);                  
}

void SPI_transmit(uint32_t 	SPI_id, QueueHandle_t SPI_txq, TickType_t xTicksToWait){
    uint16_t ch = 0;

    while (xQueueReceive(SPI_txq, &ch, xTicksToWait) == pdPASS) {
        spi_send(SPI_id, ch);        //Escribe el registro una ves que la transferencia actual haya finalizado (SPI Data Write with Blocking)
        vTaskDelay(100);
    }
}

BaseType_t SPI_receive(uint32_t SPI_id, QueueHandle_t SPI_rxq, int cant_elementos, TickType_t xTicksToWait) {
    BaseType_t status = pdPASS;  // Para controlar si todas las transmisiones son exitosas
    uint16_t ch = 0;

    for (int i = 0; i < cant_elementos; i++) {
        ch = spi_read(SPI_id);  // Leer un elemento del SPI

        // Enviar el dato a la cola. Si falla, se sale del loop
        if (xQueueSendToBack(SPI_rxq, &ch, xTicksToWait) != pdPASS) {
            status = pdFAIL;
            break;  // Si falla, no seguimos intentando
        }
    }

    return status;  // Devolver el estado, pdPASS si todos los elementos fueron encolados
}


void taskSPI_transmit(void *pvParameters) {
    char *cadena = "Prueba envio SPI1\n";
    int length = strlen(cadena);
    uint16_t data[length];  // Array de uint16_t para almacenar los datos

    // Convertir cada carácter a uint16_t
    for (int i = 0; i < length; i++) {
        data[i] = (uint16_t)cadena[i];  // Conversión de tipo en C
    }
    enqueue_SPI_data(data, length, SPI1_txq);
    for (;;) {
        SPI_transmit(SPI1, SPI1_txq, pdMS_TO_TICKS(100));
        enqueue_SPI_data(data, length, SPI1_txq);
        vTaskDelay(pdMS_TO_TICKS(500)); // Para darle tiempo a arduino a imprimir
    }
}


void enqueue_SPI_data(uint16_t* data, int length, QueueHandle_t SPI_queue) {
    for (int i = 0; i < length; i++) {
        // Intentar encolar cada elemento del vector
        if (xQueueSend(SPI_queue, &data[i], portMAX_DELAY) != pdPASS) {
            // Manejar el error de cola aquí
            //print_uart("Error al encolar datos SPI\n\r");
            break;  // Salir si hay un error
        }
    }
}


uint8_t dequeue_i2c_data(QueueHandle_t queue) {
    uint8_t data;
    if (xQueueReceive(queue, &data, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        print_uart("Error al desencolar datos SPI\n\r");
    }
    return data;
}

