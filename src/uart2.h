#ifndef UART2_H
#define UART2_H

#include "task.h"
#include "uart1.h"
#include <queue.h>
#include "semphr.h"


#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

extern SemaphoreHandle_t uart2_mutex; // Mutex for UART

void taskUART2_transmit(void *args __attribute__((unused)));
void taskUART2_receive(void *args __attribute__((unused)));

char *UART2_get_buffer(void);
uint16_t UART2_puts(const char *s);
void UART2_putchar(char ch);
void UART2_setup(void);

#endif /* ifndef UART2_H */