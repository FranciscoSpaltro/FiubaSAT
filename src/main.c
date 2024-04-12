#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// Declaración del prototipo de la función taskUART
void taskUART(void *pvParameters);
static void taskBlink(void *args __attribute__((unused)));

static void blink_setup(void) {
    // Enable clock for GPIO channel C
    rcc_periph_clock_enable(RCC_GPIOC);

    // Set pinmode for PC13
	gpio_set_mode(
		GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);

	// Turn LED off
	gpio_set(GPIOC, GPIO13);
}

static void uart_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // UART TX on PA9 (GPIO_USART1_TX)
    gpio_set_mode(GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO_USART1_TX);
    
    usart_set_baudrate(USART1,115200);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_mode(USART1,USART_MODE_TX);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

/* Handler in case our application overflows the stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)),
    char *pcTaskName __attribute__((unused))) {
        
	for (;;);
}

/* Task that sends characters through UART */
void taskUART(void *pvParameters) {
    char *message = (char *)pvParameters;
    
    while(*message) {
        usart_send_blocking(USART1, *message);
        message++;
    }
    
    return;
}

/* Task that toggles PC13, which is the LED */
static void taskBlink(void *args __attribute__((unused))) {
    for (;;) {
		gpio_toggle(GPIOC, GPIO13);
        taskUART("Fausti puto\n");
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

/* Main loop, this is where our program starts */
int main(void) {
    // Setup main clock, using external 8MHz crystal 
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    uart_setup();
    blink_setup();
    
    // Crear tarea para parpadear el LED
	xTaskCreate(taskBlink, "LED", 100, NULL, 2, NULL);

    // Start RTOS Task scheduler
	vTaskStartScheduler();

    // The task scheduler is blocking, so we should never come here...
	for (;;);
	return 0;
}