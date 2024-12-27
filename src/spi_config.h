#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

#include <stdio.h>
#include <stdint.h>  
#include <libopencm3/stm32/gpio.h>

/**
 * @brief Estructura para definir un dispositivo slave en SPI.
 *
 * Esta estructura contiene la información necesaria para controlar un slave
 * en una comunicación SPI. Cada slave está asociado a un pin de control de
 * chip select (CS), el cual se controla a través de un puerto GPIO específico.
 */
typedef struct {
    uint8_t slave_id;      /**< Identificador del slave (por ejemplo, SLAVE_1, SLAVE_2). */
    uint32_t gpio_port;    /**< Puerto GPIO correspondiente para el pin CS (por ejemplo, GPIOA, GPIOB). */
    uint16_t gpio_pin;     /**< Pin específico para el control de CS (por ejemplo, GPIO_PIN_4). */
} slave_t;


/**
 * @def SLAVE_1
 * @brief Identificador del primer slave en el bus SPI.
 */
#define SLAVE_1 1

/**
 * @def SLAVE_2
 * @brief Identificador del segundo slave en el bus SPI.
 */
#define SLAVE_2 2

/**
 * @def SLAVE_END
 * @brief Marca el final de una lista de slaves.
 *
 * Esta macro define un valor sentinela que marca el final de un arreglo de slaves.
 */
#define SLAVE_END {NULL, NULL, NULL}

// Definiciones de slaves predefinidos para SPI1
/**
 * @brief Lista de slaves predefinidos para el bus SPI1.
 *
 * Este arreglo contiene los slaves que están conectados al bus SPI1.
 * Cada elemento de la lista tiene un identificador de slave, el puerto GPIO y el pin para controlar el CS.
 * La lista termina con el valor SLAVE_END.
 */
static const slave_t spi1_slaves[] = {
    {SLAVE_1, GPIOA, GPIO4},  /**< SLAVE_1 conectado al puerto GPIOA, pin 3. */
    //{SLAVE_2, GPIOA, GPIO4},  /**< SLAVE_2 conectado al puerto GPIOA, pin 4. */
    SLAVE_END                 /**< Final de la lista de slaves. */
};

// Definiciones de slaves predefinidos para SPI2
/**
 * @brief Lista de slaves predefinidos para el bus SPI2.
 *
 * Este arreglo contiene los slaves que están conectados al bus SPI2.
 * Cada elemento de la lista tiene un identificador de slave, el puerto GPIO y el pin para controlar el CS.
 * La lista termina con el valor SLAVE_END.
 */
static const slave_t spi2_slaves[] = {
    {SLAVE_1, GPIOB, GPIO12},  /**< SLAVE_1 conectado al puerto GPIOB, pin 12. */
    SLAVE_END                  /**< Final de la lista de slaves. */
};

#endif // SPI_CONFIG_H
