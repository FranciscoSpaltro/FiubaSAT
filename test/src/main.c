#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "i2c.h"
#include "test_i2c.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// Handle para la tarea del parpadeo
static TaskHandle_t blink_handle;

/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}


int main(void) {
    //rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    blink_setup();
    if(UART_setup(USART1, 115200) != pdPASS) return -1;

    if(i2c_setup(I2C1) != I2C_PASS) return -1;

    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART1 TX", 128, (void *)USART1, 3, NULL);
    xTaskCreate(taskBlink, "LED", 100, NULL, 3, &blink_handle);

    xTaskCreate(test_request_i2c, "I2C RQ", 256, (void *) I2C1, 2, NULL);
    
    // Iniciar el planificador de FreeRTOS
    vTaskStartScheduler();
    

    while (1) {
        // El planificador de FreeRTOS no debería retornar aquí
    }

    return 0;
}