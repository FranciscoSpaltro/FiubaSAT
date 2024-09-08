#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"

void spi_setup(void);
void taskSPI_transmit_receive(void *args __attribute__((unused)));
void SPI_transmit(uint32_t 	SPI_id, QueueHandle_t SPI_txq, TickType_t xTicksToWait);
BaseType_t SPI_receive(uint32_t SPI_id, QueueHandle_t SPI_rxq, int cant_elementos, TickType_t xTicksToWait);


#endif
