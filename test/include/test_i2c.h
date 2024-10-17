#ifndef I2C_TEST_H
#define I2C_TEST_H

#include "i2c.h"
#include "htu21d.h"

#define I2C_ARDUINO_ADDRESS 0x04 // Dirección del esclavo (Arduino)
#define I2C_RASPI_ADDRESS 0x08 // Dirección del esclavo (Raspberry Pi)

void test_request_i2c(void *pvParameters);
void print_uart(const char *s);

#endif /* ifndef I2C_TEST_H */