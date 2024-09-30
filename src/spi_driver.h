#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"
#include "spi_config.h"

void taskSPI1_transmit(void *pvParameters);
void SPI_transmit(uint32_t SPI_id, TickType_t xTicksToWait);
BaseType_t SPI_receive(uint32_t SPI_id, TickType_t xTicksToWait);
BaseType_t enqueue_SPI_data(uint32_t SPI_id ,uint16_t data);
BaseType_t spi_setup(uint32_t SPI_id);
uint16_t spi_xfer_blocking(uint32_t spi, uint16_t data);

// Función para seleccionar el slave (habilitar su CS)
void spi_select_slave(uint32_t SPI_id, uint32_t SLAVE_id);

// Función para deseleccionar el slave (deshabilitar su CS)
void spi_deselect_slave(uint32_t SPI_id, uint32_t SLAVE_id);

#endif
