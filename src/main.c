#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"
#include "i2c.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"
#include "test.h"

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
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    blink_setup();
    if(UART_setup(USART1, 115200) != pdPASS) return -1;
    i2c_setup();

    xTaskCreate((TaskFunction_t)taskUART_transmit, "UART1 TX", 128, (void *)USART1, 2, NULL);
    xTaskCreate(taskBlink, "LED", 100, NULL, 2, &blink_handle);
    xTaskCreate(task_i2c, "I2C Task", 128, NULL, 1, NULL);
    
    UART_puts(USART1, "\n\r-------------------------------\n\r", pdMS_TO_TICKS(500));
    
    // Iniciar el planificador de FreeRTOS
    vTaskStartScheduler();
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        // El planificador de FreeRTOS no debería retornar aquí
    }

    return 0;
}