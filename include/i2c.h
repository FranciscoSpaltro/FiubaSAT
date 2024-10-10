#ifndef I2C_H
#define I2C_H

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include "portmacro.h"

#define I2C_ARDUINO_ADDRESS 0x04 // Dirección del esclavo (Arduino)
#define I2C_HTU21D_ADDRESS 0x40 // Dirección del sensor de humedad y temperatura
#define I2C_TIMEOUT_MS 100   // Tiempo de espera en milisegundos
#define I2C_MAX_LIST 100    // Máximo de elementos en la lista de mensajes

/**
 * @brief Estructura para manejar el periférico I2C
 * 
 * @param i2c_id Identificador del periférico I2C (I2C1 o I2C2)
 * @param txq Cola de mensajes para transmitir datos
 * @param rxq Cola de mensajes para recibir datos
 * @param mutex Semáforo para controlar el acceso a los registros del periférico
 * @param request Semáforo binario para solicitar acceso al periférico
 */

typedef struct {
    uint32_t i2c_id;
    QueueHandle_t responses;
    SemaphoreHandle_t mutex;
} i2c_t;

extern i2c_t i2c1;
extern i2c_t i2c2;

typedef enum {
    I2C_PASS,
    I2C_FAIL,
    I2C_TIMEOUT,
    I2C_EMPTY_QUEUE,
    I2C_FULL_QUEUE,
    I2C_MUTEX_FAIL
} i2c_status_t;

i2c_status_t enqueue_data(QueueHandle_t queue, uint8_t *data);
BaseType_t dequeue_i2c_response(QueueHandle_t queue, uint8_t *data);
i2c_t * get_i2c(uint32_t i2c_id);
bool i2c_wait_until_ready(uint32_t i2c_id);
bool i2c_wait_until_start(uint32_t i2c_id);
bool i2c_wait_until_address(uint32_t i2c_id);
BaseType_t i2c_start(uint32_t i2c_id, uint8_t addr, bool read);
BaseType_t i2c_write(uint32_t i2c_id, uint8_t data);
bool i2c_read(uint32_t i2c_id, uint8_t * data, bool last);
BaseType_t i2c_setup(uint32_t i2c_id);
BaseType_t i2c_make_request(uint32_t i2c_id, uint8_t addr, size_t length);
BaseType_t i2c_send_data_slave(uint32_t i2c_id, uint8_t addr, uint8_t* data, size_t length);
BaseType_t print_i2c(const char *data, uint8_t addr);


#endif /* ifndef I2C_H */