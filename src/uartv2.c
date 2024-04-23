#include "FreeRTOS.h"
#include "task.h"
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static QueueHandle_t uart_txq; // TX queue for UART
static void uart_setup(void);
static inline void uart_putc(char ch);
static void uart_task(void *args __attribute__((unused)));
static void demo_task(void *args __attribute__((unused)));

int main(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Blue pill
    // PC13:
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);
    uart_setup();
    xTaskCreate(uart_task,"UART",100,NULL,configMAX_PRIORITIES-1,NULL);
    xTaskCreate(demo_task,"DEMO",100,NULL,configMAX_PRIORITIES-2,NULL);
    vTaskStartScheduler();

    for (;;);

    return 0;
}

static void uart_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // UART TX on PA9 (GPIO_USART1_TX)
    gpio_set_mode(GPIOA,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,GPIO_USART1_TX);
    usart_set_baudrate(USART1,38400);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_mode(USART1,USART_MODE_TX);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
    
    // Create a queue for data to transmit from UART
    uart_txq = xQueueCreate(256,sizeof(char));
}

static inline void uart_putc(char ch){
    usart_send_blocking(USART1,ch);
}

//Handler in case our application overflows the stack
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
        
	for (;;);
}

static void uart_task(void *args __attribute__((unused))) {
    char ch;
    for (;;) {
        // Receive char to be TX
        if ( xQueueReceive(uart_txq,&ch,500) == pdPASS ) {
            while ( !usart_get_flag(USART1,USART_SR_TXE) )
                taskYIELD(); // Yield until ready
            usart_send(USART1,ch);
        }
        // Toggle LED to show signs of life
        gpio_toggle(GPIOC,GPIO13);
    }
}

static void uart_puts(const char *s) {
    for ( ; *s; ++s ){
        // blocks when queue is full
        xQueueSend(uart_txq,s,portMAX_DELAY);
    }
}

//*********************************************************************
//    * Demo Task:
//    * Simply queues up two line messages to be TX, one second apart.
//
//*********************************************************************

static void demo_task(void *args __attribute__((unused))) {
    for (;;) {
        uart_puts("Now this is a message..\n\r");
        uart_puts(" sent via FreeRTOS queues.\n\n\r");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}