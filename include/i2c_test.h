#ifndef I2C_TEST_H
#define I2C_TEST_H

#include "i2c.h"
#include "htu21d.h"

void test_request_i2c(void *pvParameters);
void print_uart(const char *s);
void test_i2c(void *pvParameters);
void i2c_testing_trama(void *pvParameters);

#endif /* ifndef I2C_TEST_H */