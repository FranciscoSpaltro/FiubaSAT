#include "FreeRTOS.h"
#include "task.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

static void uart_setup(void){
    // Se habilitan dos clock systemas: uno para el pin que hace de salida TX de WART1 y otro para el UART1 propiamente dicho
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // UART TX on PA9 (GPIO_USART1_TX)
    // Se utiliza la tasa más alta para permitir señales con cambios más abruptos
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    // GPIO_USART1_TX equivale al pin PA0 en la librería de libopencm3

    usart_set_baudrate(USART1, 115200);
    // Se pueden enviar 8 o 9 bits de datos, 7 y 8 si se activa la paridad
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    // Se habilita solo el modo de transmisión
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    // No se utiliza control de flujo
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

static inline void uart_putc(char ch){
    usart_send_blocking(USART1, ch);
}

static void task1(void *args __attribute__((unused))) {
    int c = '0' - 1;
    for(;;){
        // Se toggle el PC13 (built-in LED) para ver que el programa sigue funcionando
        gpio_toggle(GPIOC, GPIO13);
        // Delay para no usar control de flujo por ahora
        vTaskDelay(pdMS_TO_TICKS(200));
        if(++c >= 'z') {
            uart_putc(c);
            uart_putc('\r');
            uart_putc('\n');
            c = '0' - 1;
        } else {
            uart_putc(c);
        }
    }
}

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    uart_setup();

    xTaskCreate(task1, "task1", 100, NULL, configMAX_PRIORITIES - 1, NULL);

    vTaskStartScheduler();

    for(;;);
    return 0;
}
/********************************************************
COMENTARIOS
* usart_send_blocking() no devuelve el control hasta que USART está listo para recibir más datos. En el próximo ejemplo, veremos cómo hacerlo con un enfoque más amistoso
**********************************************************/