#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// Cuando FreeRTOS detecte que se ha superado un límite de stack, va a invocar a esta función
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

static void task1(void *args __attribute((unused))){
    for(;;){
        gpio_toggle(GPIOC, GPIO13); // Togglea el estado on/off del GPIO PC13
        vTaskDelay(pdMS_TO_TICKS(500)); // Espera 500ms
    }
}

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Establece la velocidad de clock del CPU
    rcc_periph_clock_enable(RCC_GPIOC); // Habilita el clock del GPIOC
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    xTaskCreate(task1,  // Crea la tarea task1
    "LED",  // Le da el nombre simbólico de "LED"
    100,    // Cuántas palabras de stack se requieren para el espacio de stack
    NULL,   // Puntero a cualquier dato que queramos pasarle a la tarea. Este puntero se le pasa al argumento args en task1()
    configMAX_PRIORITIES - 1,   // Le damos la prioridad más alta   
    NULL);  // Si le damos un puntero, permite devolver un task handle

    vTaskStartScheduler();  // Recién acá se empiezan a ejecutar las tareas

    for(;;);
    return 0;   // Nunca se va a devolver este valor
}
/********************************************************
COMENTARIOS
* vTaskDelay() requiere el número de ticks que se deben esperar. Frecuentemente, es más conveniente especificar los milisegundos. Para eso, se utiliza pdMS_TO_TICKS() que convierte milisegundos a ticks según nuestra configuración de FreeRTOS
* rcc_clock_setup_in_hse_8mhz_out_72mhz() establece la velocidad de clock del CPU. Para nuestra Blue Pill, normalmente queremos invocar esta función para una mejor performance. Configura el clock para que el HSE (high-speed external oscillator) use un cristal de 8 MHz, multiplicado por 9 por un PLL (phase-locked loop) para llegar a una velocidad de clock de CPU de 72 MHz. Sin esta función, deberíamos confiar en el clock RC (resistor/capacitor)
* Para la plataforma STM32, una palabra son 4 bytes. Estimar el espacio de stack es complicado, por ahora aceptar que 400 bytes es suficiente
* vApplicationStackOverflowHook() permite al diseñador de la aplicación decidir que debería hacerse (por ejemplo, encender un LED especial que indique que el programa falló). __attribute((unused)) es un atributo de gcc que le indica al compilador que la variable no se usa y que no debería quejarse por ello
* La función main llama a vTaskStartScheduler() y le da el control al schehduler de FreeRTOS, que arranca y switchea entre varias tareas. Esto sigue hasta que una tarea detenga el scheduler
* La tarea task1() tiene un stack asignado desde el heap (le dimos 100 palabras). Si la tarea fuera eliminada, este almacenamiento sería liberado al heap
**********************************************************/