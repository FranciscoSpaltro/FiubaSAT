/*
RECORDATORIOS: dar opcion a menos manual activando el CS o AUTOMATICO (lo maneja el periferico)
-En caso de tener un solo esclavo es mejor el uso en automatico. Que pasa si queda el el chip enabkle y se va a otra tarea?
- En select y deselect salve manejar los errores
-Ver que pasa cuando se desconecta el spi
NO puse lo de las tareas, porque en el momento que salga de esta tarea, pasa a otra y queda activado el chip select
*/

#include "FreeRTOS.h"
#include "spi_driver.h"
#include <libopencm3/stm32/rcc.h>
#include <string.h>

#define DUMMY   0xFFFF
#define SHIFT_8 8

typedef struct {
    uint32_t SPI_id;  // SPI_id 
    QueueHandle_t SPI_txq;  // Cola de transmisión
    QueueHandle_t SPI_rxq;  // Cola de recepción donde se bufferean los datos
    SemaphoreHandle_t mutex;  // Mutex para protección de acceso
    slave_t *slaves; // Arreglo de esclavos
    uint8_t data_size;           // Tamaño de los datos (8 para 8 bits, 16 para 16 bits)
} spi_t;

// Definición de estructuras SPI_t
static spi_t spi1;
static spi_t spi2;

//Prototipos de funciones
static spi_t *get_spi(uint32_t SPI_id);
static BaseType_t spi_create(spi_t *spi, uint32_t SPI_id, uint8_t data_size, Mode_t mode);
static const slave_t* spi_get_slave(const spi_t *spi, uint8_t slave_id);
static void spi_slaves_init(slave_t *slaves);

// Manejadores de SPIs

static spi_t *get_spi(uint32_t SPI_id) {
    switch (SPI_id) {
        case SPI1: return &spi1;
        case SPI2: return &spi2;
        default: return NULL;
    }
}

// Inicialización de la estructura spi_t
static BaseType_t spi_create(spi_t *spi, uint32_t SPI_id, uint8_t data_size, Mode_t mode){

    spi->SPI_id = SPI_id;  // Asigna el SPI_id correspondiente

    spi->mutex = xSemaphoreCreateMutex();

    xSemaphoreGive(spi->mutex);

    // Asigna el puntero a los esclavos según el SPI_id
    if (mode == master_mode){

        if (SPI_id == SPI1) {
            spi->slaves = spi1_slaves;  // Usar la configuración de SPI1
        }
        else if (SPI_id == SPI2){
            spi->slaves = spi2_slaves;
        }        
    }
    
    else {
        spi->slaves = NULL;
    }   

    spi->data_size = data_size;

    return pdPASS;
}

//Inicializa los pines de los esclavos en modo de salida
static void spi_slaves_init(slave_t *slaves){    

    for (size_t i = 0; slaves[i].slave_id != NULL; i++) {

        gpio_set_mode(slaves[i].gpio_port,
            GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_PUSHPULL,
            slaves[i].gpio_pin); 

        gpio_set(slaves[i].gpio_port, slaves[i].gpio_pin);
    }
}

// Función para buscar el esclavo por ID a partir de un puntero a spi_t
static const slave_t* spi_get_slave(const spi_t *spi, uint8_t slave_id) {
    // Iterar sobre los esclavos en el arreglo dentro de la estructura spi_t
    for (size_t i = 0; spi->slaves[i].slave_id != NULL; i++) {
        if (spi->slaves[i].slave_id == slave_id) {
            return &spi->slaves[i]; // Retorna un puntero al esclavo encontrado
        }
    }
    return NULL; // No se encontró el esclavo
}

BaseType_t spi_setup(uint32_t SPI_id, Mode_t mode) {

    spi_t *spi = get_spi(SPI_id);       //Obtengo estructura spi_t en base al SPI_id
    
    if (SPI_id == SPI1){
        if(spi_create(spi, SPI1, DATA_SIZE_8, mode) != pdPASS) return pdFAIL;

        rcc_periph_clock_enable(RCC_SPI1); //Enable the clock for SPI1

        spi_init_master(
            SPI1,
            SPI_CR1_BAUDRATE_FPCLK_DIV_64,     //Se debe tener en cuenta la maxima frecuencia de operacion de APB1 y APB2
            SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    //SCK en estado bajo en el estado inactivo (IDLE)
            SPI_CR1_CPHA_CLK_TRANSITION_1,      //Determina la fase del CLK (en que flanco se captura)
            SPI_CR1_DFF_8BIT,                   //Indica el largo de palabra a utilizar
            SPI_CR1_MSBFIRST                    //Se transmite el MSB primero
        );

        /* Configuración de los pines GPIO para SPI en modo Maestro */
        
        if (mode == master_mode){

            gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO4 | GPIO5 | GPIO7);  // NSS=PA4, SCK=PA5, MOSI=PA7

            gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO6);  // MISO=PA6

            spi_slaves_init(spi->slaves);
        }

        else {
            gpio_set_mode(GPIOA,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO4 | GPIO5 | GPIO7);  // NSS=PA4, SCK=PA5, MOSI=PA7

            gpio_set_mode(GPIOA,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO6);  // MISO=PA6

            spi_set_slave_mode(SPI1);

            spi_enable_rx_buffer_not_empty_interrupt(SPI1);
            nvic_enable_irq(NVIC_SPI1_IRQ);
        }

    }

    else if (SPI_id == SPI2){

        if(spi_create(spi, SPI2, DATA_SIZE_8, mode) != pdPASS) return pdFAIL;

        rcc_periph_clock_enable(RCC_SPI2); //Enable the clock for SPI1
        
        spi_init_master(
            SPI2,
            SPI_CR1_BAUDRATE_FPCLK_DIV_256,     //Se debe tener en cuenta la maxima frecuencia de operacion de APB1 y APB2
            SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    //SCK en estado bajo en el estado inactivo (IDLE)
            SPI_CR1_CPHA_CLK_TRANSITION_1,      //Determina la fase del CLK (en que flanco se captura)
            SPI_CR1_DFF_8BIT,                   //Indica el largo de palabra a utilizar
            SPI_CR1_MSBFIRST                    //Se transmite el MSB primero
        );

        if (mode == master_mode){
            gpio_set_mode(
                GPIOB,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO12 | GPIO13 | GPIO15 // PB12=NSS2 PB13=SCK2 PB15=MOSI2 
            );

            gpio_set_mode(
                GPIOB,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO14 // MISO2=PB14
            );
            spi_slaves_init(spi->slaves);
        }

        else {
            gpio_set_mode(
                GPIOB,
                GPIO_MODE_INPUT,
                GPIO_CNF_INPUT_FLOAT,
                GPIO12 | GPIO13 | GPIO15 // PB12=NSS2 PB13=SCK2 PB15=MOSI2 
            );

            gpio_set_mode(
                GPIOB,
                GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                GPIO14 // MISO2=PB14
            );
            spi_set_slave_mode(SPI2);

            spi_enable_rx_buffer_not_empty_interrupt(SPI2);
            nvic_enable_irq(NVIC_SPI2_IRQ);
        }        
    }

    spi_disable_software_slave_management(spi->SPI_id);    //Desactivo el manejo del NSS por software

    if (mode == slave_mode){
        spi_disable_ss_output(spi->SPI_id);    //Configuro el pin NSS entrada.
        
    }

    else {
        spi_enable_ss_output(spi->SPI_id);    //Configuro el pin NSS como salida.
    }
    
    spi_enable(spi->SPI_id);
    
    return pdTRUE;                  
}

//REVISAR EL MANEJO DE ERRORES
void spi_set_dff(uint32_t spi_id, uint8_t data_size){
    spi_t *spi = get_spi(spi_id);       //Obtengo estructura spi_t en base al SPI_id
    if (spi == NULL) return;

    if (data_size == DATA_SIZE_8){
        spi_set_dff_8bit(spi->SPI_id);
    }
    else spi_set_dff_16bit(spi->SPI_id);
}

// Función para seleccionar el slave (habilitar su CS)
BaseType_t spi_select_slave(uint32_t spi_id, uint32_t slave_id){
    spi_t *spi = get_spi(spi_id);       //Obtengo estructura spi_t en base al SPI_id
    if (spi == NULL) return pdFALSE;

    // Intentar tomar el mutex con un timeout de 100ms
    if (xSemaphoreTake(spi->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        slave_t *slave = spi_get_slave(spi, slave_id);
        
        if (slave == NULL){
            return pdFALSE;
        }
        gpio_clear(slave->gpio_port, slave->gpio_pin);
       
    } else {
        // Error al tomar el mutex (posible timeout)
        // Manejar el error si es necesario
        return pdFALSE;
    }

    return pdPASS;
}

// Función para deseleccionar el slave (deshabilitar su CS)
BaseType_t spi_deselect_slave(uint32_t spi_id, uint32_t slave_id){
    spi_t *spi = get_spi(spi_id);       //Obtengo estructura spi_t en base al SPI_id
    slave_t *slave = spi_get_slave(spi, slave_id);

    if (slave == NULL){
            return pdFALSE;
    }
    gpio_set(slave->gpio_port, slave->gpio_pin);

    if (xSemaphoreGive(spi->mutex) != pdTRUE){
        // No se esperaría que esta llamada falle porque debimos haber obtenido el semáforo antes de llegar aquí.
        return pdFALSE;
    };
    return pdPASS;    
}

void spi_transmit(uint32_t SPI_id, uint8_t *data, uint16_t size, TickType_t xTicksToWait) {
    spi_t *spi = get_spi(SPI_id);
    if (spi == NULL) return;
    
    TickType_t start = xTaskGetTickCount();

    if (spi->data_size == DATA_SIZE_8){
        for (int i = 0; i < size; i++) {
            /* Espera a que termine la transferencia o el timeout */
            while (!(SPI_SR(SPI_id) & SPI_SR_TXE)) {
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                    // Ver manejo de error 
                    return;
                }
            }

            /* Escribe el dato en el registro DR */
            SPI_DR(SPI_id) = data[i];
      
            /* Wait for transfer finished. */
            while (!(SPI_SR(SPI_id) & SPI_SR_RXNE)){
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                        // Ver manejo de error 
                        return;
                }
            }

            /* Read the data (8 or 16 bits, depending on DFF bit) from DR. */
            SPI_DR(SPI_id);     //Lectura dummy para vaciamiento del buffer de recepcion
            
        }
    }

    else if (spi->data_size == DATA_SIZE_16){
        // Lo tratamos como puntero de datos de 16 bits
        uint16_t *txdata16 = (uint16_t *)data;

        for (int i = 0; i < size; i++) {
            /* Espera a que termine la transferencia o el timeout */
            while (!(SPI_SR(SPI_id) & SPI_SR_TXE)) {
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                    // Ver manejo de error 
                    return;
                }
            }

            SPI_DR(SPI_id) = txdata16[i];
                        
            /* Wait for transfer finished. */
            while (!(SPI_SR(SPI_id) & SPI_SR_RXNE)){
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                        // Ver manejo de error 
                        return;
                }
            }

            /* Read the data (8 or 16 bits, depending on DFF bit) from DR. */
            SPI_DR(SPI_id);     //Lectura dummy para vaciamiento del buffer de recepcion
            
        }
    }    
}

void spi_transmit_receive(uint32_t SPI_id, uint8_t *txdata, uint16_t *rxdata, uint16_t size, TickType_t xTicksToWait) {

    spi_t *spi = get_spi(SPI_id);
    if (spi == NULL) return;
    
    TickType_t start = xTaskGetTickCount();

    if (spi->data_size == DATA_SIZE_8){
        for (int i = 0; i < size; i++) {
            /* Espera a que termine la transferencia o el timeout */
            while (!(SPI_SR(SPI_id) & SPI_SR_TXE)) {
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                    // Ver manejo de error 
                    return;
                }
            }

            /* Escribe el dato en el registro DR */
            SPI_DR(SPI_id) = txdata[i];
      
            /* Wait for transfer finished. */
            while (!(SPI_SR(SPI_id) & SPI_SR_RXNE)){
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                        // Ver manejo de error 
                        return;
                }
            }

            /* Read the data (8 or 16 bits, depending on DFF bit) from DR. */
            rxdata[i] = SPI_DR(SPI_id);     
            
        }
    }

    else if (spi->data_size == DATA_SIZE_16){
        // Lo tratamos como puntero de datos de 16 bits
        uint16_t *txdata16 = (uint16_t *)txdata;

        for (int i = 0; i < size; i++) {
            /* Espera a que termine la transferencia o el timeout */
            while (!(SPI_SR(SPI_id) & SPI_SR_TXE)) {
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                    // Ver manejo de error 
                    return;
                }
            }

            SPI_DR(SPI_id) = txdata16[i];
                        
            /* Wait for transfer finished. */
            while (!(SPI_SR(SPI_id) & SPI_SR_RXNE)){
                if ((xTaskGetTickCount() - start) > xTicksToWait) {
                        // Ver manejo de error 
                        return;
                }
            }

            /* Read the data (8 or 16 bits, depending on DFF bit) from DR. */
            rxdata[i] = SPI_DR(SPI_id);     
            
        }
    }    
}

void spi_receive(uint32_t SPI_id, uint16_t *data, uint16_t size, TickType_t xTicksToWait) {
    spi_t *spi = get_spi(SPI_id);
    if (spi == NULL) return;
    
    TickType_t start = xTaskGetTickCount();

    for (int i = 0; i < size; i++) {
        /* Wait for transfer finished. */
        while (!(SPI_SR(SPI_id) & SPI_SR_TXE)){
            if ((xTaskGetTickCount() - start) > xTicksToWait) {
                // Ver manejo de error //
                return;
            }
        };

        /* Write data (8 or 16 bits, depending on DFF) into DR. */
        SPI_DR(SPI_id) = DUMMY;     //Envio de valor dummy para generacion de señal de clock
        
        /* Wait for transfer finished. */
	    while (!(SPI_SR(SPI_id) & SPI_SR_RXNE)){            
            if ((xTaskGetTickCount() - start) > xTicksToWait) {
                // Ver manejo de error //
                return;
            }
        }
        
        /* Read the data (8 or 16 bits, depending on DFF bit) from DR. */
        data[i] = SPI_DR(SPI_id);        
            
    }    
}
