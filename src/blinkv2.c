/*
#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void gpio_setup(void);

static void task1(void *args __attribute((unused))){
    for(;;){
        gpio_toggle(GPIOC,GPIO13);  //LED off 
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void){
    
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // For "blue pill"

    //rcc_periph_clock_enable(RCC_GPIOC);
    //gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);

    gpio_setup();
    
    xTaskCreate(task1,"LED",100,NULL,configMAX_PRIORITIES-1,NULL);
    vTaskStartScheduler();
    for (;;);
    
    return 0;
}

//Configuracion del puerto C 

static void gpio_setup(void){
    //Enable GPIOC clock.
    rcc_periph_clock_enable(RCC_GPIOC);

    //Set GPIO8 (in GPIO port C) to 'output push-pull'.
    gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL,GPIO13);
    }


//Handler in case our application overflows the stack
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
        
	for (;;);
}

*/

/*******************WORKFLOW DE CONFIGURACIÓN DE INPUT PINS************************/
/*This applies to GPIO inputs only—not a peripheral input, like the USART. Peripherals require 
other considerations, especially if alternate pin configurations are involved (they will be 
covered later in the book).
1. Enable the GPIO port clock ---> rcc_periph_clock_enable(RCC_GPIOC)
2. Set the mode of the input pin with gpio_set_mode(), specifying the port in argument one, and 
the GPIO_MODE_INPUT macro in argument two.
3. Choose the appropriate specialization macro GPIO_CNF_INPUT_ANALOG, GPIO_CNF_INPUT_FLOAT, or 
GPIO_INPUT_PULL_UPDOWN as appropriate.
4. Finally, specify in the last argument all pin numbers that apply. These are or-ed together, 
as in GPIO12|GPIO15, for example.*/

