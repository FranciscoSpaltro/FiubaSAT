#ifndef SPI_CONFIG_H
#define SPI_CONFIG_H

#include <stdint.h>  
#include <libopencm3/stm32/gpio.h>

// Estructura para definir un dispositivo slave en SPI
typedef struct {
    uint8_t slave_id;      // Identificador del slave (por ejemplo, SLAVE_1, SLAVE_2, etc.)
    uint32_t gpio_port;    // Puerto GPIO correspondiente para el pin CS (por ejemplo, GPIOA, GPIOB)
    uint16_t gpio_pin;     // Pin específico para el control de CS (por ejemplo, GPIO_PIN_4)
} slave_t;

// Definiciones de macros para los diferentes slaves
#define SPI1_SLAVE_COUNT 3
#define SPI2_SLAVE_COUNT 2
#define SLAVE_1 1
#define SLAVE_2 2
#define SLAVE_END {0, 0, 0}

// Definiciones de slaves predefinidos para SPI1

static const slave_t spi1_slaves[SPI1_SLAVE_COUNT] = {
    {SLAVE_1, GPIOA, GPIO3},  // SLAVE_1 en GPIOA, pin 3
    {SLAVE_2, GPIOA, GPIO4},  // SLAVE_2 en GPIOA, pin 4
    SLAVE_END
};

//HACER LA FUNCIO SPI_get
static const slave_t spi2_slaves[SPI2_SLAVE_COUNT] = {
    //{SLAVE_1, GPIOA, 1},  // SLAVE_1 en GPIOA, pin 1
    {SLAVE_2, GPIOB, GPIO12},  // SLAVE_2 en GPIOA, pin 4
    SLAVE_END
};



#endif // SPI_CONFIG_H
