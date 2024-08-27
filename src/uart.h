#ifndef UART_H
#define UART_H

#include "task.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define SIZE_BUFFER 256  // Define el tamaño del buffer

// Identificadores para los UARTs
#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

// Declaraciones de funciones
void UART_setup(uint32_t usart, uint32_t baudrate);
void taskUART_transmit(uint32_t usart_id);
void taskUART_receive(uint32_t usart_id);
int UART_receive(uint32_t usart_id);
uint16_t *UART_get_buffer(uint32_t usart_id);
uint16_t UART_puts(uint32_t usart_id, const char *s);
void UART_putchar(uint32_t usart_id, uint16_t ch);
void usart_generic_isr(uint32_t usart_id);
bool UART_buffer_read(uint32_t usart_id, uint16_t *data);
void UART_print_buffer(uint32_t usart_id);

#endif /* ifndef UART_H */