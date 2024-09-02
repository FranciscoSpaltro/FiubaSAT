static void spi_setup(void) {
    rcc_periph_clock_enable(RCC_SPI1); //Enable the clock for SPI1
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_OUTPUT_50_MHZ,
        GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
        GPIO4 | GPIO5 | GPIO7 // NSS=PA4, SCK=PA5, MOSI=PA7
    );
    gpio_set_mode(
        GPIOA,
        GPIO_MODE_INPUT,
        GPIO_CNF_INPUT_FLOAT,
        GPIO6 // MISO=PA6
    );

    spi_reset(SPI1);

    spi_init_master(
        SPI1,
        SPI_CR1_BAUDRATE_FPCLK_DIV_256,     //Se debe tener en cuenta la maxima frecuencia de operacion de APB1 y APB2
        SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_1,
        SPI_CR1_DFF_8BIT,
        SPI_CR1_MSBFIRST
    );
    spi_disable_software_slave_management(SPI1);
    spi_enable_ss_output(SPI1);
}


