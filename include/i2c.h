#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

#define I2C_SLAVE_ADDRESS 0x08 // Dirección del esclavo (Arduino)
#define I2C_TIMEOUT_MS 1000   // Tiempo de espera en milisegundos

void i2c_setup(void);
void i2c_wait_until_ready(void);
void i2c_start(uint8_t addr, bool read);
void i2c_write(uint8_t data);
uint8_t i2c_read(bool last);
void i2c_stop(void);
void task_i2c(void *pvParameters);
void i2c_reset(void);


/******************************
 * TESTING PARA I2C
 * ***************************/
 
 void print_uart(const char *s);