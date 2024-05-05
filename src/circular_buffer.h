#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define SIZE_BUFFER_USART1 256

typedef struct {
    volatile uint8_t data[SIZE_BUFFER_USART1];
    uint16_t head;
    uint16_t tail;
    uint16_t size;
} CircularBuffer_t;

void buffer_push(CircularBuffer_t *buffer, uint8_t data);
uint8_t buffer_pop(CircularBuffer_t *buffer);
bool buffer_is_empty(CircularBuffer_t *buffer);
bool buffer_is_full(CircularBuffer_t *buffer);

#endif /* CIRCULAR_BUFFER_H */
