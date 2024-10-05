#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "test.h"
#include "circular_buffer.h"
#include "uart.h"
#include "NMEA.h"

#include "libopencm3/stm32/rcc.h"

#define SIZE_BUFFER_GPS 512
static int get_id(uint32_t usart_id) {
    if (usart_id == USART1) {
        return 1;
    } else if (usart_id == USART2) {
        return 2;
    } else if (usart_id == USART3) {
        return 3;
    } else {
        return -1;
    }
}

void test_uart_available_data(uint32_t usart_id) {
    char msg[60];
    int id = get_id(usart_id);

    // Se deshabilita la interrupción de recepción
    usart_disable_rx_interrupt(usart_id);

    // Se limpia la cola de recepción
    UART_clear_rx_queue(usart_id, pdMS_TO_TICKS(100));
    // Reseteado el buffer de resepción y sin interrupción activa debería haber 0 datos disponibles
    if (UART_available_data(usart_id) == 0) {
        snprintf(msg, sizeof(msg), "Test UART%i available data runned successfully\r\n", id);
        UART_puts(USART3, msg, pdMS_TO_TICKS(100));
    } else {
        snprintf(msg, sizeof(msg), "Test UART%i available data failed\r\n", id);
        UART_puts(USART3, msg, pdMS_TO_TICKS(100));
    }

    // Se habilita la interrupción de recepción
    usart_enable_rx_interrupt(usart_id);
}

void test_gps_registros(uint32_t usart_id) {
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

void taskTest(void *args __attribute__((unused))){
    // Espero 100 ms para que se envíen los datos
    test_uart_available_data(USART1);
    test_uart_available_data(USART2);
    test_uart_available_data(USART3);
    vTaskDelay(pdMS_TO_TICKS(1000));
    test_gps_registros(USART1);

    vTaskDelete(NULL);
}