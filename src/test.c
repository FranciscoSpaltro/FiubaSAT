#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "test.h"
#include "circular_buffer.h"

#include "libopencm3/stm32/rcc.h"

#define SIZE_BUFFER_GPS 512

void taskTest(void *args __attribute__((unused))){
    // Espero 100 ms para que se envíen los datos
    vTaskDelay(pdMS_TO_TICKS(2000));

    taskTestGPS(USART1);
    vTaskDelete(NULL);
}

void taskTestGPS(uint32_t usart_id) {
    char buffer[SIZE_BUFFER_GPS]; // Buffer para almacenar los datos a enviar
    char GGA[100]; // Buffer para almacenar el mensaje GGA
    char RMC[100]; // Buffer para almacenar el mensaje RMC

    int flagGGA = 0, flagRMC = 0;

    get_rxq_buffer(usart_id, buffer, SIZE_BUFFER_GPS); // Leer datos de la cola de recepción
    if((flagGGA = Copy_from_to(buffer, "$GPGGA,", "*", GGA, 100)) != 0) {
        UART_puts(USART3, "Error al copiar los datos de GGA\r\n", pdMS_TO_TICKS(100));
    }
    if((flagRMC = Copy_from_to(buffer, "$GPRMC,", "*", RMC, 100)) != 0) {
        UART_puts(USART3, "Error al copiar los datos de RMC\r\n", pdMS_TO_TICKS(100));
    }
    
    if (flagGGA == 0 && flagRMC == 0) {
        UART_puts(USART3, "Test GPS runned successfully\r\n", pdMS_TO_TICKS(100));
    }
    
    UART_clear_rx_queue(usart_id, pdMS_TO_TICKS(100));
}

void taskPrintBuffer(void *args __attribute__((unused))) {
    for (;;) {
        UART_print_buffer(USART1);
        UART_print_buffer(USART2);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void taskTestUART_Semaphore(void *args __attribute__((unused))) {
    // Se espera a que haya datos disponibles en la cola de recepción
    UART_semaphore_release(USART1);

    if(UART_semaphore_take(USART1, pdMS_TO_TICKS(100)) == pdFALSE){
        UART_puts(USART3, "Test Semaphore failed. No se pudo tomar.\r\n", pdMS_TO_TICKS(100));
        UART_semaphore_release(USART1);
        vTaskDelete(NULL);
        return;
    }
    
    if(UART_semaphore_take(USART1, pdMS_TO_TICKS(500)) == pdTRUE){
        UART_puts(USART3, "Test Semaphore failed. Tomado dos veces.\r\n", pdMS_TO_TICKS(100));
        UART_semaphore_release(USART1);
        vTaskDelete(NULL);
        return;
    }

    UART_puts(USART3, "Test Semaphore runned\r\n", pdMS_TO_TICKS(100));
    // Se libera el semáforo
    UART_semaphore_release(USART1);    
    vTaskDelete(NULL);
}
