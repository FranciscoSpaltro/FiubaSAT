#include "circular_buffer.h"

void buffer_push(CircularBuffer_t *buffer, uint8_t data) {
    buffer->data[buffer->head] = data;
    buffer->head = (buffer->head + 1) % buffer->size;
}

uint8_t buffer_pop(CircularBuffer_t *buffer) {
    uint8_t data = buffer->data[buffer->tail];
    buffer->tail = (buffer->tail + 1) % buffer->size;
    return data;
}

bool buffer_is_empty(CircularBuffer_t *buffer) {
    return buffer->head == buffer->tail;
}

bool buffer_is_full(CircularBuffer_t *buffer) {
    return (buffer->head + 1) % buffer->size == buffer->tail;
}
