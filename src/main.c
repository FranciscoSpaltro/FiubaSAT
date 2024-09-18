#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"
#include "test.h"
#include "spi_driver.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <string.h>

/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}


static void uart_setup(void) ;
static inline void uart_putc(char ch);
void taskSPI1_transmit(void *pvParameterss);

/* Main loop donde arranca el programa */

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Blue Pill

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);

    // Configuración del LED en PC13
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    uart_setup();       // Configuración de UART
    spi_setup(SPI1);        // Configuración del SPI

    //spi_rx = xQueueCreate(256,sizeof(char));      //Creo la cola
    
    gpio_set(GPIOC, GPIO13);        // PC13 = on
    
    
    uint8_t received_data;

    //xTaskCreate(taskSPI_transmit,"SPI trasnmit", 500,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(taskSPI1_transmit,"SPI trasnmit", 500,NULL,configMAX_PRIORITIES-1,NULL);
    vTaskStartScheduler();
    
    for (;;);
    
    return 0;
}


static void uart_setup(void) {

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // UART TX on PA9 (GPIO_USART1_TX)
    gpio_set_mode(GPIOA,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_USART1_TX);

    usart_set_baudrate(USART1, 38400);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}


static inline void uart_putc(char ch) {
    usart_send_blocking(USART1, ch);
}

// Asumiendo que uart_txq y usart_get_flag/usart_send están definidos e inicializados

void usart_Transmit(QueueHandle_t uart_txq) {
    char ch;

    for (;;) {
        // Recibir carácter para transmitir
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {
            // Esperar hasta que el registro de transmisión esté vacío
            while (!usart_get_flag(USART1, USART_SR_TXE)) {
                taskYIELD(); // Ceder tiempo de CPU hasta que esté listo
            }
            // Enviar carácter
            usart_send(USART1, ch);
        }
    }
}

