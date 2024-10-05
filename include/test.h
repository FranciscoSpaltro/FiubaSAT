// funciones.h
#ifndef TEST_H
#define TEST_H

#include "uart.h"
#include <stdint.h>

void taskTest(void *args __attribute__((unused)));
void taskTestUART_Semaphore(void *args __attribute__((unused)));
void taskTestGPS(uint32_t usart_id);
void test_uart_available_data(uint32_t usart_id);
void test_gps_registros(uint32_t usart_id);

#endif