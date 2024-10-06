#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include "uart.h"
#include "circular_buffer.h"
#include "NMEA.h"
#include "blink.h"
#include "test.h"

#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

// Handle para la tarea del parpadeo y test
TaskHandle_t blink_handle;
TaskHandle_t test_handle;

// Delay para el parpadeo
uint32_t blinkDelay = 500;

#define SIZE_BUFFER_RX1 512

/**
 * @brief Tarea manejo GPS UART1.
 * Recepción de datos NMEA desde un GPS a través de USART1.
 * Decodificación de los mensajes GGA y RMC.
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 */
static void taskUART1_GPS(uint32_t usart_id) {
    vTaskDelay(pdMS_TO_TICKS(4000)); // Espero que terminen los tests

    char buffer[SIZE_BUFFER_RX1];   // Buffer para almacenar los datos recibidos

    char GGA[100]; // Buffer para almacenar el mensaje GGA
    char RMC[100]; // Buffer para almacenar el mensaje RMC

    GPSSTRUCT gpsData; // Estructura para almacenar los datos decodificados

    int flagGGA = 0, flagRMC = 0; // Flags para indicar si los datos son válidos
    char bufferNMEA[100]; // Buffer para almacenar los datos a enviar

    for (;;) {
        memset(buffer, 0, SIZE_BUFFER_RX1);
        if (UART_available_data(usart_id) > 0) {
            usart_disable_rx_interrupt(usart_id);
            get_rxq_buffer(usart_id, buffer, SIZE_BUFFER_RX1); // Leer datos de la cola de recepción
            
            Copy_from_to(buffer, "$GPGGA,", "*", GGA, 100);
            if (decodeGGA(GGA, &gpsData.ggastruct) == 0) flagGGA = 2;  // 2 indicates the data is valid
            else flagGGA = 1;  // 1 indicates the data is invalid

            Copy_from_to(buffer, "$GPRMC,", "*", RMC, 100);
            if (decodeRMC(RMC, &gpsData.rmcstruct) == 0) flagRMC = 2;  // 2 indicates the data is valid
            else flagRMC = 1;  // 1 indicates the data is invalid
            
             
            if (flagGGA == 2 && flagRMC == 2) {
                UART_puts(USART3, "GGA and RMC data is valid\r\n", pdMS_TO_TICKS(100));
                snprintf(bufferNMEA, sizeof(bufferNMEA), "Latitud: %s %c - Longitud: %s %c - %i/%i/%i\r\n", gpsData.ggastruct.lcation.latitude, 
                        gpsData.ggastruct.lcation.NS, gpsData.ggastruct.lcation.longitude, gpsData.ggastruct.lcation.EW, 
                        gpsData.rmcstruct.date.Day, gpsData.rmcstruct.date.Mon, gpsData.rmcstruct.date.Yr);
                UART_puts(USART3, bufferNMEA, pdMS_TO_TICKS(500));
            } 
            else {
                UART_puts(USART3, "GGA and RMC data is invalid\r\n", pdMS_TO_TICKS(100));
                UART_puts(USART3, buffer, pdMS_TO_TICKS(100));
                UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
                UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            }       

            UART_puts(USART3, GGA, pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            
            UART_puts(USART3, RMC, pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));

            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));
            UART_puts(USART3, "\r\n", pdMS_TO_TICKS(100));

            UART_puts(USART1, buffer, pdMS_TO_TICKS(100));

            UART_clear_rx_queue(usart_id, pdMS_TO_TICKS(100));
            usart_enable_rx_interrupt(usart_id);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Tarea para la transmisión de datos a través de UART.
 * Lee los datos de la cola de recepción de usart_id y los vuelve a enviar a través de este mismo.
 * @param usart_id Identificador del USART (USART1, USART3, USART3).
 */
static void taskUART_echo(uint32_t usart_id) {
    uint8_t data;
    for (;;) {
        // Esperar a que el semáforo indique que hay datos disponibles
        if (UART_semaphore_take(usart_id, portMAX_DELAY) == pdTRUE) {
            // Procesar todos los datos en la cola
            while (UART_receive(usart_id, &data)) {
                // Aquí puedes manejar el dato recibido (por ejemplo, almacenarlo o procesarlo)
                UART_putchar(usart_id, data, pdMS_TO_TICKS(100));
            }
            // Liberar el semáforo después de procesar los datos
            UART_semaphore_release(usart_id);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

/* Main loop donde arranca el programa */
int main(void) {
    // Setup main clock, using external 8MHz crystal 
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    // Inicialización del LED para el blink
    blink_setup();
    
    // Inicialización de UARTs con sus baudrates
    if(UART_setup(USART1, 9600) != pdPASS) return -1;
    if(UART_setup(USART2, 115200) != pdPASS) return -1;
    if(UART_setup(USART3, 115200) != pdPASS) return -1;

    // Creación de tareas genéricas para transmisión UART
    xTaskCreate(taskUART_transmit, "UART1 TX", 256, (void *)USART1, 2, NULL);
    xTaskCreate(taskUART_transmit, "UART2 TX", 256, (void *)USART2, 2, NULL);
    xTaskCreate(taskUART_transmit, "UART3 TX", 256, (void *)USART3, 2, NULL);
    
    // Crear tareas para Test
    xTaskCreate(taskTest, "Test", 128, (void *)test_handle, 2, &test_handle);  // Crear tarea para Test
    
    // Crear tarea para parpadear el LED
    xTaskCreate(taskBlink, "LED", 128, (void *)blinkDelay, 2, &blink_handle);  // Crear tarea para parpadear el LED

    // Creación de tareas genéricas para recepción UART
    xTaskCreate(taskUART1_GPS, "UART1 GPS", 1024, (void *)USART1, 2, NULL);
    xTaskCreate(taskUART_echo, "UART2 Echo", 128, (void *)USART2, 2, NULL);

    // Start RTOS Task scheduler
	vTaskStartScheduler();

    // The task scheduler is blocking, so we should never come here...
	for (;;);
    
	return 0;
}