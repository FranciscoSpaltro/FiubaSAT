#include "FreeRTOS.h"
#include "uart.h"
#include <stdio.h>

static QueueHandle_t uart_txq; // TX queue for UART
static QueueHandle_t uart_rxq; // RX queue for UART

SemaphoreHandle_t uart_mutex;

char *message1 = "Recibiendo por RX\n";

void UART_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // -> GPIO
    gpio_set_mode(GPIO_BANK_USART1_TX, 
        GPIO_MODE_OUTPUT_50_MHZ, 
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
        GPIO_USART1_TX);

    gpio_set_mode(GPIO_BANK_USART1_RX, 
        GPIO_MODE_INPUT, 
        GPIO_CNF_INPUT_FLOAT, 
        GPIO_USART1_RX);

    usart_set_mode(USART1,USART_MODE_TX_RX);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_baudrate(USART1,115200);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);

    // Habilitar la UART
    usart_enable(USART1);

    // Habilitar la interrupción de recepción de la UART
    usart_enable_rx_interrupt(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);

    // Create a queue for data to transmit from UART
    uart_txq = xQueueCreate(256,sizeof(char));
    uart_rxq = xQueueCreate(256,sizeof(char));

    // Create a mutex for UART
    uart_mutex = xSemaphoreCreateBinary();
    if(uart_mutex == NULL) {
        UART_puts("ERROR SEMAFOROOOO\n");
    }
    else {
        UART_puts("ANDUVO\n");
    }
    xSemaphoreGive(uart_mutex);
}

void taskUART(void *args __attribute__((unused))) {
    char ch;
    for (;;) {
        // Receive char to be TX
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {
            while (!usart_get_flag(USART1,USART_SR_TXE) )
                taskYIELD(); // Yield until ready
            usart_send_blocking(USART1,ch);
        }

        // Esperar a que haya datos en la cola RXQ
        /*
        uint16_t data;
        char message[7];
        if (xQueueReceive(uart_rxq, &data, portMAX_DELAY) == pdTRUE) {
            // Hacer algo con los datos recibidos, como enviarlos a través de USART nuevamente
            sprintf(message, "%u\n", data);
            if(xSemaphoreTake(uart_mutex, portMAX_DELAY) == pdTRUE) {
                UART_puts(message);
                xSemaphoreGive(uart_mutex);
            }
        }
        */
    }
}


void UART_puts(const char *s) {
    for ( ; *s; ++s ) {
        // blocks when queue is full
        if(xQueueSend(uart_txq, s, portMAX_DELAY) != pdTRUE) {
            xQueueReset(uart_txq);
            return; // Queue full
        }
    }
}

void usart1_isr() {
    /*
    
    */
    if (usart_get_flag(USART1, USART_SR_RXNE)) {
        uint16_t data = usart_recv(USART1);  // Leer el byte recibido
        if(xQueueSendToBackFromISR(uart_rxq, &data, NULL) != pdTRUE) { // Encolar el byte en RXQ
            xQueueReset(uart_rxq);
        } 
    }
}