#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void gpio_setup(void)
{
    // Enable GPIOC clock
    rcc_periph_clock_enable(RCC_GPIOC);
    
    // Set GPIO13 (in GPIO port C) to 'output push-pull' mode
    gpio_set_mode(GPIOC, // Puerto GPIO afectado
    GPIO_MODE_OUTPUT_2_MHZ, // INPUT - OUTPUT y velocidad
    GPIO_CNF_OUTPUT_PUSHPULL, // Especialización del puerto
    GPIO13); //pin afectado
}

int main(void) {
    int i;
    gpio_setup();
    for(;;){
        // Prender el led
        gpio_clear(GPIOC, GPIO13);
        
        // Esperar
        for(i = 0; i < 3000000; i++)
            __asm__("nop");

        // Apagar el led
        gpio_set(GPIOC, GPIO13);

        // Esperar
        for(i = 0; i < 1500000; i++)
            __asm__("nop");
    }

    return 0; // Nunca se ejecuta pero es necesario para que no se queje el compilador
}

/********************************************************
COMENTARIOS
* rcc.h tiene las definiciones que permiten habilitar el GPIO clock
* gpio.h tiene las definiciones que permiten configurar los pines del GPIO (gpio_set, gpio_clear, etc)
* A gpio_clear() y gpio_set() se le pasan dos argumentos: el puerto (GPIO port name) y el pin (GPIO pin number)
* para gpi_set_mode(): la velocidad es qué tan rápido el pin responderá al cambio. ESTO AFECTA CONSUMO Y EMI
* __asm__("nop") es una instrucción que no hace nada. Si no estuviera, el compilador optimizaría el loop y no se vería el parpadeo
* El problema con el delay programado es que
    * no es preciso (¿cuantas iteraciones necesito?)
    * no es portable (el delay varía según la plataforma, el CPU clock rate y el contexto de ejecución)
    * desperdicia recursos del CPU que podrían usarse en un ambiente multi-tasking
    * no es confiable cuando se usa preemptive multitasking (el sistema operativo puede interrumpir el loop en cualquier momento)
    
    Depende, entonces, del CPU clock rate, la cantidad de instrucciones que se ejecutan por segundo y si estamos en un ambiente multitasking o no
**********************************************************/