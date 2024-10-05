#ifndef UART_H
#define UART_H

#include "task.h"
#include <queue.h>
#include "semphr.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

/**
 * @brief Configura el periférico USART.
 * 
 * Inicializa el periférico USART con el baudrate especificado.
 * 
 * @param usart Identificador del USART (USART1, USART2, USART3).
 * @param baudrate Tasa de baudios para la comunicación.
 * @return pdPASS si la configuración fue exitosa, pdFAIL en caso contrario.
 */
BaseType_t UART_setup(uint32_t usart, uint32_t baudrate);

/**
 * @brief Tarea que transmite datos a través de UART.
 * 
 * Tarea que se encarga de transmitir datos a través del USART especificado.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 */
void taskUART_transmit(uint32_t usart_id);

// Nota: El encolamiento de datos en la cola de RX se realiza en la interrupción USART_ISR.

/**
 * @brief Recibe un dato desde la cola de recepción de UART.
 * 
 * Lee un dato de la cola de recepción que fue llenada por la interrupción USART_ISR.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param data Puntero a la variable donde se almacenará el dato recibido.
 * @return pdPASS si la recepción fue exitosa, pdFAIL en caso contrario.
 */
BaseType_t UART_receive(uint32_t usart_id, uint8_t *data);

/**
 * @brief Limpia la cola de recepción de UART.
 * 
 * Elimina todos los datos en la cola de recepción del USART especificado.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param xTicksToWait Tiempo de espera para limpiar la cola.
 * @return pdPASS si la limpieza fue exitosa, pdFAIL en caso contrario.
 */
BaseType_t UART_clear_rx_queue(uint32_t usart_id, TickType_t xTicksToWait);

/**
 * @brief Encola un string en la cola de transmisión de UART.
 * 
 * Envía una cadena de caracteres a través del USART especificado, bloqueando la tarea 
 * si la cola está llena.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param s Cadena de caracteres a enviar.
 * @param xTicksToWait Tiempo de espera para encolar el string.
 * @return Número de caracteres encolados exitosamente.
 */
uint16_t UART_puts(uint32_t usart_id, const char *s, TickType_t xTicksToWait);

/**
 * @brief Encola un dato en la cola de transmisión de UART.
 * 
 * Envía un carácter a través del USART especificado, bloqueando la tarea si la cola 
 * está llena.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param ch Carácter a enviar.
 * @param xTicksToWait Tiempo de espera para encolar el carácter.
 * @return pdPASS si el carácter se encoló exitosamente, pdFAIL en caso contrario.
 */
BaseType_t UART_putchar(uint32_t usart_id, uint8_t ch, TickType_t xTicksToWait);

/**
 * @brief Imprime el contenido del buffer de UART.
 * 
 * Imprime todos los datos de la cola de recepción del USART especificado a través de 
 * otra UART para su visualización.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 */
void UART_print_buffer(uint32_t usart_id);

/**
 * @brief Espera a que haya datos disponibles en la cola de recepción.
 * 
 * Intenta tomar el semáforo para acceder a la cola de recepción del USART especificado.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param ticks_to_wait Tiempo de espera para tomar el semáforo.
 * @return pdTRUE si se tomó el semáforo exitosamente, pdFALSE en caso contrario.
 */
BaseType_t UART_semaphore_take(uint32_t usart_id, TickType_t ticks_to_wait);

/**
 * @brief Libera el semáforo de acceso a la cola de transmisión.
 * 
 * Libera el semáforo para permitir el acceso a la cola de transmisión del USART 
 * especificado.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @return pdPASS si se liberó el semáforo exitosamente, pdFAIL en caso contrario.
 */
BaseType_t UART_semaphore_release(uint32_t usart_id);

/**
 * @brief Devuelve el buffer de recepción de UART.
 * 
 * Copia el contenido del buffer de recepción del USART especificado a un buffer de 
 * salida.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param output Buffer de salida para almacenar los datos.
 * @param max_len Longitud máxima del buffer de salida.
 * @return Número de datos copiados al buffer de salida.
 */
uint16_t get_rxq_buffer(uint32_t usart_id, char *output, uint16_t max_len);

/**
 * @brief Devuelve la cantidad de datos disponibles en la cola de recepción.
 * 
 * Calcula el número de datos disponibles en la cola de recepción del USART especificado.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @return Número de datos disponibles en la cola de recepción.
 */
uint16_t UART_available_data(uint32_t usart_id);

#endif  // UART_H