#ifndef I2C_TEST_H
#define I2C_TEST_H

#include "i2c.h"
#include "htu21d.h"

#define I2C_ARDUINO_ADDRESS 0x04 // Dirección del esclavo (Arduino)
#define LIFE_LINE_PIN GPIO4
#define LIFE_LINE_PORT GPIOB
#define LIFE_LINE_RCC RCC_GPIOB
#define LIFE_LINE_DELAY 1000 // 500 ms

void life_line_setup(void);
void vLifeLineTask(void *pvParameters);
void test_request_i2c(void *pvParameters);
void print_uart(const char *s);

#endif /* ifndef I2C_TEST_H */