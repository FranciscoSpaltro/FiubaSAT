#ifndef UART_H
#define UART_H

#include "task.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

extern SemaphoreHandle_t uart_mutex; // Mutex for UART

void UART_puts(const char *s);
void UART_setup(void);
void taskUART(void *args __attribute__((unused)));

#endif /* ifndef UART_H */