#ifndef UART_H
#define UART_H

#include "task.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

// Configura el periférico USART
void UART_setup(uint32_t usart, uint32_t baudrate);

// Tarea que transmite datos a través de UART1
void taskUART_transmit(uint32_t usart_id);

// OBS: el encolamiento de datos en la cola de RX se realiza en la interrupción USART_ISR

// Recibe un dato desde la cola de recepción de UART, que fue llenada por USART_ISR
BaseType_t UART_receive(uint32_t usart_id, uint16_t *data, TickType_t xTicksToWait);

// Devuelve el buffer de recepción de UART
uint16_t *UART_get_buffer(uint32_t usart_id);

// Encola un string en uart_txq, bloqueando la tarea si la cola está llena
uint16_t UART_puts(uint32_t usart_id, const char *s);

// Encola el dato en uart_txq, bloqueando la tarea si la cola está llena
void UART_putchar(uint32_t usart_id, uint16_t ch);

// Imprime el contenido del buffer de UART
void UART_print_buffer(uint32_t usart_id);

// Espera a que haya datos disponibles en la cola de recepción
BaseType_t UART_semaphore_take(uint32_t usart_id, TickType_t ticks_to_wait);

// Libera el semáforo de acceso a la cola de transmisión
void UART_semaphore_release(uint32_t usart_id);

#endif