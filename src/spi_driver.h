#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"
#include <queue.h>
#include "semphr.h"
#include "spi_config.h"

#define DATA_SIZE_8 8
#define DATA_SIZE_16 16

typedef enum {
    master_mode, slave_mode
} Mode_t;

void taskSPI1_transmit(void *pvParameters);

BaseType_t SPI_receive(uint32_t SPI_id, TickType_t xTicksToWait);
BaseType_t enqueue_SPI_data(uint32_t SPI_id ,uint16_t data);
BaseType_t spi_setup(uint32_t SPI_id, Mode_t mode);
uint16_t spi_xfer_blocking(uint32_t spi, uint16_t data);
void spi_set_dff(uint32_t spi_id, uint8_t data_size);
void spi_transmit(uint32_t SPI_id, void *data, uint16_t size, TickType_t xTicksToWait);
void spi_receive(uint32_t SPI_id, uint16_t *data, uint16_t size, TickType_t xTicksToWait);
// Función para seleccionar el slave (habilitar su CS)
BaseType_t spi_select_slave(uint32_t SPI_id, uint32_t SLAVE_id);
void uart_puts(const char *str);
// Función para deseleccionar el slave (deshabilitar su CS)
BaseType_t spi_deselect_slave(uint32_t SPI_id, uint32_t SLAVE_id);

#endif
