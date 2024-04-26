#include "FreeRTOS.h"
#include "uart.h"

static QueueHandle_t uart_txq; // TX queue for UART

void UART_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // UART TX RX on PA9 (GPIO_USART1_TX)
    gpio_set_mode(GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART1_TX|GPIO_USART1_RX);
    
    usart_set_baudrate(USART1,115200);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_mode(USART1,USART_MODE_TX_RX);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
    usart_enable(USART1);

    // Create a queue for data to transmit from UART
    uart_txq = xQueueCreate(256,sizeof(char));
}

void taskUART(void *args __attribute__((unused))) {
    char ch;
    for (;;) {
        // Receive char to be TX
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {
            while (!usart_get_flag(USART1,USART_SR_TXE) )
                taskYIELD(); // Yield until ready
            usart_send(USART1,ch);
        }
    }
}

void UART_puts(const char *s) {
    for ( ; *s; ++s ) {
        // blocks when queue is full
        if(xQueueSend(uart_txq, s, portMAX_DELAY) != pdTRUE) {
            break; // Queue full
        }
    }
}