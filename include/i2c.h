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

// HTU21D
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define SOFT_RESET  0xFE
#define HTU21D_ADDRESS 0x40

BaseType_t i2c_setup(uint32_t i2c_id);
void test_request_i2c(void *pvParameters);



/******************************
 * TESTING PARA I2C
 * ***************************/
 
void print_uart(const char *s);
#endif /* ifndef I2C_H */