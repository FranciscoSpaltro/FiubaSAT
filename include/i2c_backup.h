#ifndef I2C_H
#define I2C_H
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "uart.h"
#include "portmacro.h"

#define I2C_SLAVE_ADDRESS 0x04 // Dirección del esclavo (Arduino)
#define I2C_TIMEOUT_MS 1000   // Tiempo de espera en milisegundos

#define I2C_MAX_BUFFER 5

BaseType_t i2c_setup(uint32_t i2c_id);
void task_i2c_tx(void *pvParameters);
void task_read_i2c(void *pvParameters);
void test_write_i2c(void *pvParameters);
void test_request_i2c(void *pvParameters);

uint8_t * i2c_make_request(uint32_t i2c_id, msg_t msg);

/******************************
 * TESTING PARA I2C
 * ***************************/
 
void print_uart(const char *s);
#endif /* ifndef I2C_H */