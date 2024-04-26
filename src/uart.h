#ifndef UART_H
#define UART_H

#include "task.h"
#include <queue.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

void UART_puts(const char *s);
void UART_setup(void);
void taskUART(void *args __attribute__((unused)));

#endif /* ifndef UART_H */