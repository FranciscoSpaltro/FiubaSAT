/*
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void uart_setup(void);
static void task1(void *args __attribute__((unused)));
static inline void uart_putc(char ch);

int main(void) {

    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Blue pill
    // PC13:
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);
    uart_setup();
    xTaskCreate(task1,"task1",100,NULL,configMAX_PRIORITIES-1,NULL);
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
    
}

static void task1(void *args __attribute__((unused))){
    char miString[] = "FiubaSAT FreeRTOS STM32";
    int i = -1;
    uart_putc('\r');
    uart_putc('\n');

    for (;;) {
        gpio_toggle(GPIOC,GPIO13);
        vTaskDelay(pdMS_TO_TICKS(200));

        if ( miString[++i] == '\0' ) {
            uart_putc('\r');
            uart_putc('\n');
            i = - 1; 
        }
        else{
            uart_putc(miString[i]);
        }
    }
}

static inline void uart_putc(char ch){
    usart_send_blocking(USART1,ch);
}

//Handler in case our application overflows the stack
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
        
	for (;;);
}

*/