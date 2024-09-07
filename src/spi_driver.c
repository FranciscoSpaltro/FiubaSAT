#include "FreeRTOS.h"
#include "spi_driver.h"
#include <libopencm3/stm32/rcc.h>

QueueHandle_t SPI_rxq; //Creo la cola de recepcion
QueueHandle_t SPI_txq; //Creo la cola de recepcion

#define SPI_SIZE_BUFFER 256  // Queues size

void spi_setup(void) {
    SPI_rxq = xQueueCreate(SPI_SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de recepción
    SPI_txq = xQueueCreate(SPI_SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de transmisión

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
       vTaskDelay(pdMS_TO_TICKS(50));   //Only for debug        
    }
}

BaseType_t SPI_receive(uint32_t SPI_id, QueueHandle_t SPI_rxq, TickType_t xTicksToWait){
    uint16_t ch = 0;
    ch = spi_read(SPI_id);

    return xQueueSendToBack(SPI_rxq, &ch, xTicksToWait);
}



