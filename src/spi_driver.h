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

/**
 * @brief Enumeración para definir el modo SPI.
 */
typedef enum {
    master_mode, /**< Modo maestro. */
    slave_mode   /**< Modo esclavo. */
} Mode_t;

/**
 * @brief Configura el bus SPI.
 * 
 * Esta función inicializa el bus SPI con una configuración predeterminada que incluye:
 * - Prescaler establecido en 64 para ajustar la frecuencia del reloj SPI.
 * - Orden de transmisión de datos configurado para enviar el bit más significativo (MSB) primero.
 * - Configuración del modo de operación 0 (CPOL = 0, CPHA = 0).
 * 
 * En caso de que no se haya definido ningún esclavo, la señal de Chip Select (CS) será gestionada
 * automáticamente por el periférico en el pin correspondiente.
 * 
 * @param SPI_id Identificador del bus SPI, que puede ser SPI1 o SPI2.
 * @param mode Modo de operación, que puede ser `master_mode` o `slave_mode`.
 * @return `pdTRUE` si la configuración fue exitosa, o `pdFAIL` en caso de error.
 */
BaseType_t spi_setup(uint32_t SPI_id, Mode_t mode);

/**
 * @brief Configura el tamaño de los datos a transmitir/recibir por SPI.
 * 
 * @param spi_id Identificador del bus SPI.
 * @param data_size Tamaño del dato, puede ser `DATA_SIZE_8` para 8 bits o `DATA_SIZE_16` para 16 bits.
 */
void spi_set_dff(uint32_t spi_id, uint8_t data_size);

/**
 * @brief Transmite un bloque de datos por SPI de forma bloqueante.
 * 
 * @param SPI_id Identificador del bus SPI (SPIx).
 * @param data Puntero al bloque de datos a transmitir.
 * @param size Tamaño del bloque de datos.
 * @param xTicksToWait Tiempo en ticks que la tarea esperará en caso de timeout.
 */
void spi_transmit(uint32_t SPI_id, uint8_t *data, uint16_t size, TickType_t xTicksToWait);

/**
 * @brief Recibe un bloque de datos por SPI de forma bloqueante.
 * 
 * @param SPI_id Identificador del bus SPI (SPIx).
 * @param data Puntero al buffer donde se almacenarán los datos recibidos.
 * @param size Tamaño del bloque de datos a recibir.
 * @param xTicksToWait Tiempo en ticks que la tarea esperará si la cola está vacía.
 */
void spi_receive(uint32_t spi_id, uint16_t *data, uint16_t size, TickType_t xTicksToWait);

/**
 * @brief Selecciona el esclavo en el bus SPI, activando su pin CS.
 * 
 * @param SPI_id Identificador del bus SPI.
 * @param SLAVE_id Identificador del esclavo en el bus SPI.
 * @return `pdPASS` si la selección fue exitosa, `pdFAIL` en caso de error.
 */
BaseType_t spi_select_slave(uint32_t SPI_id, uint32_t SLAVE_id);

/**
 * @brief Deselecciona el esclavo en el bus SPI, desactivando su pin CS.
 * 
 * @param SPI_id Identificador del bus SPI.
 * @param SLAVE_id Identificador del esclavo en el bus SPI.
 * @return `pdPASS` si la deselección fue exitosa, `pdFAIL` en caso de error.
 */
BaseType_t spi_deselect_slave(uint32_t SPI_id, uint32_t SLAVE_id);

#endif // SPI_DRIVER_H
