#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"

void spi_setup(uint32_t SPI_id);
void taskSPI1_transmit(void *pvParameters);
void SPI_transmit(uint32_t SPI_id, TickType_t xTicksToWait);
BaseType_t SPI_receive(uint32_t SPI_id, TickType_t xTicksToWait);
BaseType_t enqueue_SPI_data(uint32_t SPI_id ,uint16_t data);
void spi_setup(uint32_t SPI_id);

#endif
