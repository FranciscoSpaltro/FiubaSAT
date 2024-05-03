#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static QueueHandle_t uart_txq;   // TX queue for UART

void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

static void uart_setup(void){

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);

    // Creo una queue para transmitir datos por UART
    uart_txq = xQueueCreate(256, sizeof(char));
}

static void uart_task(void *args __attribute__((unused))) {
    char ch;

    for(;;){
        // Se llama a xQueueReceive() para obtener el mensaje. El último argumento es el tiempo máximo en ticks que se espera. Si pasa este tiempo sin recibir el mensaje, devuelve pdFAIL
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {
            // Si xQUeueReceive devolvió pdPASS es porque la tarea recibió un mensaje. El mensaje es recibido como un caracter único en la variable ch. Una vez que recibimos el caracter de la queue, lo enviamos por UART
            // Se testea el flag USART_SR_TXE (transmit empty) hasta que deje de indicar "not empty"
            while(!usart_get_flag(USART1, USART_SR_TXE))
                // Llamamos a taskYIELD() hasta que UART pueda aceptar otro caracter (cedemos el control a otra tarea si es necesario). Si no cedemos el control, la función usart_send_blocking() quedaría dando vueltas hasta que UART esté disponible. Si el UART no estuviera listo a tiempo, este giro consumiría tiempo de CPU hasta que se agotara la porción de tiempo de esa tarea.
                taskYIELD();

            // Una vez que el flag TXE está en 1, se envía el caracter
            usart_send(USART1, ch);
        }

        // Togglear el LED para mostrar signos de vida
        gpio_toggle(GPIOC, GPIO13);
    }
}

static void uart_puts(const char *s) {
    for(; *s; s++) {
        // Bloquea cuando la queue está llena
        xQueueSend(uart_txq, s, portMAX_DELAY);
        // Recibe un puntero a una cadena de caracteres (const char *s) como argumento. Itera sobre la cadena de caracteres utilizando un bucle for, y en cada iteración encola el caracter actual en la cola de mensajes uart_txq utilizando la función xQueueSend(). Si la cola está llena en ese momento, la función se bloquea hasta que haya espacio disponible en la cola (portMAX_DELAY indica que la función esperará indefinidamente hasta que haya espacio en la cola). La función termina cuando ha recorrido toda la cadena de caracteres y los ha encolado en la cola de mensajes.
    }
}
// ENCOLA DOS LINEAS DE MENSAJES PARA SER TX, UN SEGUNDO DESPUES DE CADA UNA
static void demo_task(void *args __attribute__((unused))) {
    for(;;) {
        uart_puts("Now this is a message..\n\r");
        uart_puts(" sent via FreeRTOS queues.\n\n\r");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

    uart_setup();

    xTaskCreate(uart_task, "UART", 100, NULL, configMAX_PRIORITIES - 1, NULL);

    xTaskCreate(demo_task, "DEMO", 100, NULL, configMAX_PRIORITIES - 2, NULL);

    vTaskStartScheduler();

    for(;;);
    return 0;
}
/********************************************************
COMENTARIOS
* QueueHandle_t crea una message queue que contendrá un máximo de 256 mensajes, cada uno de largo 1 byte. La variable uart_txq recibe entonces un handle válido
* demo_task() llama a la rutina uart_puts() para enviar cadenas de texto por UART, con un segundo de diferencia. uart_puts() invoca a uart_putc() para encolar los caracteres en una message queue referenciada por el handle uart_txq. Si la cola esta llena, el control 
* uart_task() y demo_task() se ejecutan de forma concurrente
* demo_task() envía mensajes a la cola uart_txq, y uart_task() los envía por UART

* En resumen:
    * La tarea demo_task() llama a la rutina uart_puts() para enviar cadenas de texto al UART, con una separación de un segundo entre ellas.
    * La función uart_puts() invoca uart_putc() para encolar los caracteres en una cola de mensajes referenciada por el handle uart_txq. Si la cola está llena, la tarea demo_task() cede el control.
    * La tarea uart_task() desencola los caracteres recibidos de la cola referenciada por el handle uart_txq. Cada caracter recibido se envía al UART para ser enviado, siempre y cuando esté listo. Cuando el UART está ocupado, la tarea cede el control.
**********************************************************/