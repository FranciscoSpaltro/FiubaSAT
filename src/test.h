// funciones.h
#ifndef TEST_H
#define TEST_H

#include "uart.h"
#include <stdint.h>

void taskTest(void *args __attribute__((unused)));
void taskPrintBuffer(void *args __attribute__((unused)));
void taskTestUART_Semaphore(void *args __attribute__((unused)));
void taskTestGPS(uint32_t usart_id);

#endif