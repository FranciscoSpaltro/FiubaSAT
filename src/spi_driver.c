#include "FreeRTOS.h"
#include "spi_driver.h"
#include <libopencm3/stm32/rcc.h>
#include <string.h>
#include "crc.h"

#define SPI_SIZE_BUFFER 256  // Queues size
#define DEBUG_PLOTTING

typedef struct {
    uint32_t SPI_id;  // SPI_id 
    QueueHandle_t SPI_txq;  // Cola de transmisión
    QueueHandle_t SPI_rxq;  // Cola de recepción donde se bufferean los datos
    SemaphoreHandle_t mutex;  // Mutex para protección de acceso
} spi_t;

// Definición de estructuras SPI_t
static spi_t spi1;
static spi_t spi2;

//Prototipos de funciones
static spi_t *get_spi(uint32_t SPI_id);
static BaseType_t spi_init(spi_t *spi, uint32_t SPI_id);

// Manejadores de SPIs
static spi_t *get_spi(uint32_t SPI_id) {
    switch (SPI_id) {
        case SPI1: return &spi1;
        case SPI2: return &spi2;
        default: return NULL;
    }
}

void spi_setup(uint32_t SPI_id) {

    spi_t *spi = get_spi(SPI_id);       //Obtengo estructura spi_t en base al SPI_id
    
    if (SPI_id == SPI1){
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

        //spi_reset(SPI1);
        
        spi_init_master(
            SPI1,
            SPI_CR1_BAUDRATE_FPCLK_DIV_256,     //Se debe tener en cuenta la maxima frecuencia de operacion de APB1 y APB2
            SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    //SCK en estado bajo en el estado inactivo (IDLE)
            SPI_CR1_CPHA_CLK_TRANSITION_1,      //Determina la fase del CLK (en que flanco se captura)
            SPI_CR1_DFF_8BIT,                   //Indica el largo de palabra a utilizar
            SPI_CR1_MSBFIRST                    //Se transmite el MSB primero
        );
        if(spi_init(spi, SPI1) != pdPASS) return pdFAIL;
    }

    else if (SPI_id == SPI2){
        rcc_periph_clock_enable(RCC_SPI2); //Enable the clock for SPI1
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
            GPIO12 // MISO2=PB14
        );

        //spi_reset(SPI2);
        
        spi_init_master(
            SPI2,
            SPI_CR1_BAUDRATE_FPCLK_DIV_256,     //Se debe tener en cuenta la maxima frecuencia de operacion de APB1 y APB2
            SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,    //SCK en estado bajo en el estado inactivo (IDLE)
            SPI_CR1_CPHA_CLK_TRANSITION_1,      //Determina la fase del CLK (en que flanco se captura)
            SPI_CR1_DFF_8BIT,                   //Indica el largo de palabra a utilizar
            SPI_CR1_MSBFIRST                    //Se transmite el MSB primero
        );

        if(spi_init(spi, SPI2) != pdPASS) return pdFAIL;
        
    }

    // Me puedo desentender del codigo

    spi_disable_software_slave_management(spi->SPI_id);    //Desactivo el manejo del NSS por software
    spi_enable_ss_output(spi->SPI_id);    //Activo el manejo del NSS por hardware (SPI PHP)
    spi_enable(spi->SPI_id);                  
}

// Inicialización de SPI
static BaseType_t spi_init(spi_t *spi, uint32_t SPI_id) {

    spi->SPI_id = SPI_id;  // Asigna el SPI_id correspondiente

    spi->SPI_rxq = xQueueCreate(SPI_SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de recepción
    if (spi->SPI_rxq == NULL) return pdFAIL;

    spi->SPI_txq = xQueueCreate(SPI_SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de transmisión
        
    if (spi->SPI_txq == NULL) {
        vQueueDelete(spi->SPI_rxq);
        return pdFAIL;
    }

    spi->mutex = xSemaphoreCreateMutex();

    if (spi->mutex == NULL) {
        vQueueDelete(spi->SPI_txq);
        vQueueDelete(spi->SPI_rxq);
        return pdFAIL;
    }

    xSemaphoreGive(spi->mutex);

    return pdPASS;
}

void SPI_transmit(uint32_t SPI_id, TickType_t xTicksToWait) {
    spi_t *spi = get_spi(SPI_id);
    if (spi == NULL) return;

    // Intentar tomar el mutex con un timeout de 100ms
    if (xSemaphoreTake(spi->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint16_t ch = 0;

        // Procesar la cola de transmisión mientras haya datos
        while (xQueueReceive(spi->SPI_txq, &ch, xTicksToWait) == pdPASS) {
            // Transmitir el dato por SPI (bloquea hasta que finalice)
            spi_send(SPI_id, ch);

            // Para debugging, retraso opcional (puede ajustarse o eliminarse)
            #ifdef DEBUG_PLOTTING
            vTaskDelay(pdMS_TO_TICKS(100));  // Retraso para debugging con Arduino
            #endif
        }

        // Liberar el mutex después de finalizar la transmisión
        xSemaphoreGive(spi->mutex);
    } else {
        // Error al tomar el mutex (posible timeout)
        // Manejar el error si es necesario
    }
}

BaseType_t SPI_receive(uint32_t SPI_id, TickType_t xTicksToWait) {
    BaseType_t status = pdPASS;  // Para controlar si todas las transmisiones son exitosas
    uint16_t ch = 0;
    
    spi_t *spi = get_spi(SPI_id);
    if (spi == NULL) return pdFAIL;

    ch = spi_read(SPI_id);  // Leer un elemento del SPI

    // Enviar el dato a la cola. Si falla, se sale del loop
    if (xQueueSendToBack(spi->SPI_rxq, &ch, xTicksToWait) != pdPASS) {
        status = pdFAIL;
        //Ver manejo de error
    }

    return status;  // Devolver el estado, pdPASS si todos los elementos fueron encolados
}

void taskSPI1_transmit(void *pvParameters) {
    char *cadena = "Prueba envio SPI1\n";
    int length = strlen(cadena);
    uint16_t data[length];  // Array de uint16_t para almacenar los datos

    // Convertir cada carácter a uint16_t
    for (int i = 0; i < length; i++) {
        data[i] = (uint16_t)cadena[i];  // Conversión de tipo en C
        enqueue_SPI_data(SPI1, data[i]);
    }
    
    for (;;) {
        SPI_transmit(SPI1, pdMS_TO_TICKS(100));

        //Vuelvo a encolar los datos
        for (int i = 0; i < length; i++){
            enqueue_SPI_data(SPI1, data[i]);
        };
        
        vTaskDelay(pdMS_TO_TICKS(500)); // Para darle tiempo a arduino a imprimir
    }
}

/*
void taskSPI2_transmit(void *pvParameters) {
    char *cadena = "Prueba envio SPI2\n";
    int length = strlen(cadena);
    uint16_t data[length];  // Array de uint16_t para almacenar los datos

    // Convertir cada carácter a uint16_t
    for (int i = 0; i < length; i++) {
        data[i] = (uint16_t)cadena[i];  // Conversión de tipo en C
    }
    enqueue_SPI_data(data, length, SPI2_txq);
    for (;;) {
        SPI_transmit(SPI2, SPI2_txq, pdMS_TO_TICKS(100));
        enqueue_SPI_data(data, length, SPI2_txq);
        vTaskDelay(pdMS_TO_TICKS(500)); // Para darle tiempo a arduino a imprimir
    }
}

void taskSPI2_receive(void *pvParameters) {
    int length = spi_xfer(SPI2,'R');
    for (;;) {
        SPI_receive(SPI2, SPI2_rxq,length, pdMS_TO_TICKS(100)); //Recibe length cantidad de datos
        Queue_send(SPI2_rxq , SPI2_txq);
        usart_transmit(SPI2, SPI2_txq,pdMS_TO_TICKS(100));         
    }
}

*/


// Implementación de la función enqueue_SPI_data
BaseType_t enqueue_SPI_data(uint32_t SPI_id, uint16_t data) {
    spi_t *spi = get_spi(SPI_id);  // Obtiene la estructura spi_t en base al SPI_id
    
    if (spi == NULL) {
        // Manejo de error si no se puede obtener la estructura spi_t
        //print_uart("Error: SPI_id no válido\n\r");
        return pdFAIL;
    }

    // Intentar encolar el dato
    if (xQueueSend(spi->SPI_txq, &data, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        //print_uart("Error al encolar datos SPI\n\r");
        return pdFAIL;
    }

    // Si todo salió bien, devolvemos pdPASS
    return pdPASS;
}

// Implementación de la función dequeue_SPI_data
BaseType_t dequeue_SPI_data(uint32_t SPI_id, uint16_t *data) {
    spi_t *spi = get_spi(SPI_id);  // Obtiene la estructura spi_t en base al SPI_id
    
    if (spi == NULL) {
        // Manejo de error si no se puede obtener la estructura spi_t
        //print_uart("Error: SPI_id no válido\n\r");
        return pdFAIL;
    }

    // Intentar desencolar el dato
    if (xQueueReceive(spi->SPI_rxq, data, portMAX_DELAY) != pdPASS) {
        // Manejar el error de cola aquí
        //print_uart("Error al desencolar datos SPI\n\r");
        return pdFAIL;
    }

    // Si todo salió bien, devolvemos pdPASS
    return pdPASS;
}

//En proceso
void SPI_transmit_with_crc(uint32_t SPI_id, TickType_t xTicksToWait) {
    spi_t *spi = get_spi(SPI_id);
    if (spi == NULL) return;
    
    uint16_t ch = 0;

    // Procesar la cola de transmisión mientras haya datos
    if(xQueueReceive(spi->SPI_txq, &ch, xTicksToWait) == pdPASS) {

        spi_set_next_tx_from_buffer(SPI_id);
        spi_send(SPI_id, ch); // Transmite el dato por SPI (bloquea hasta que finalice)
        crc_calculate(ch);
        
        spi_set_next_tx_from_crc(SPI_id);
        

        // Para debugging, retraso opcional (puede ajustarse o eliminarse)
        #ifdef DEBUG_PLOTTING
        vTaskDelay(pdMS_TO_TICKS(100));  // Retraso para debugging con Arduino
        #endif
    }
      
    else {
         // Manejar el error si es necesario
    }
}


// ------------------------------------ VERSION SIMPLE ----------------------------------------------- //


BaseType_t SPI_send(uint32_t SPI_id, uint16_t data, TickType_t xTicksToWait) {
    spi_t *spi = get_spi(SPI_id);  // Obtiene la estructura spi_t en base al SPI_id
    
    if (spi == NULL) {
        // Manejo de error si no se puede obtener la estructura spi_t
        //print_uart("Error: SPI_id no válido\n\r");
        return pdFAIL;
    }

    // Intentar encolar el dato
    if (xQueueSend(spi->SPI_txq, &data, xTicksToWait) != pdPASS) {
        // Manejar el error de cola aquí
        //print_uart("Error al encolar datos SPI\n\r");
        return errQUEUE_FULL;
    }

    // Si todo salió bien, devolvemos pdPASS
    return pdPASS;
}

// Retorna por parametros el data_read en caso de que la queue este llena y no se pueda encolar
BaseType_t SPI_receive(uint32_t SPI_id, uint16_t *data_read, TickType_t xTicksToWait) {
    BaseType_t status = pdPASS;  // Para controlar si todas las transmisiones son exitosas
        
    spi_t *spi = get_spi(SPI_id);  // Obtiene la estructura spi_t en base al SPI_id
    
    if (spi == NULL) {
        // Manejo de error si no se puede obtener la estructura spi_t
        //print_uart("Error: SPI_id no válido\n\r");
        return pdFAIL;
    }

    data_read = spi_read(SPI_id);  // Leer un elemento del SPI

    // Enviar el dato a la cola. Si falla, se sale del loop
    if (xQueueSendToBack(spi->SPI_rxq, &data_read, xTicksToWait) != pdPASS) {
        status = errQUEUE_FULL;
        //Ver manejo de error
    }

    return status;  // Devolver el estado, pdPASS si todos los elementos fueron encolados
}

void taskSPI_transmit(uint32_t SPI_id, TickType_t xTicksToWait) {

    spi_t *spi = get_spi(SPI_id);  // Obtiene la estructura spi_t en base al SPI_id
    
    if (spi == NULL) {
        // Manejo de error si no se puede obtener la estructura spi_t
        //print_uart("Error: SPI_id no válido\n\r");
        //return pdFAIL;
    }
 
    for (;;) {

        // Intentar tomar el mutex con un timeout de 100ms
        if (xSemaphoreTake(spi->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint16_t data = 0;

            // Procesar la cola de transmisión mientras haya datos
            while (xQueueReceive(spi->SPI_txq, &data, xTicksToWait) == pdPASS) {
                
                spi_send(SPI_id, data);     // Transmitir el dato por SPI (bloquea hasta que finalice)

                // Para debugging, retraso opcional (puede ajustarse o eliminarse)
                #ifdef DEBUG_PLOTTING
                vTaskDelay(pdMS_TO_TICKS(100));  // Retraso para debugging con Arduino
                #endif
            }

            if (uxQueueMessagesWaiting(spi->SPI_txq) != 0){     //Chequeo si la Queue no se vacio a causa de un error
                    //Ver gestion de error
                    //Podria vaciarse con el fin de no generar problemas con las demas tareas
            }

            // Liberar el mutex después de finalizar la transmisión y/o gestionar los errores
            xSemaphoreGive(spi->mutex);
        } else {            
            // Error al tomar el mutex (posible timeout)
            // Manejar el error si es necesario
        }
    }
}
        



// ---------------------------------------- CONSULTAS -------------------------------------------------//

typedef struct {
    uint8_t slave_id;      // ID del dispositivo slave (por ejemplo, el pin del CS)
    uint8_t *message;      // Puntero al mensaje a transmitir
    size_t message_length; // Longitud del mensaje
} spi_message_t;


typedef struct {
    uint8_t slave_id;      // ID del dispositivo slave (por ejemplo, el pin del CS)
    uint16_t message;      // Puntero al mensaje a transmitir
} spi_message_t;

typedef struct {
    uint8_t slave_id;     // Identificador del slave (por ejemplo, SLAVE_1, SLAVE_2, etc.)
    uint32_t gpioport;   // Puerto GPIO correspondiente para el pin CS
    uint16_t gpios;         // Pin específico para el control de CS
} slave_t;

typedef struct {
    uint32_t SPI_id;               // SPI_id (número de la interfaz SPI)
    QueueHandle_t SPI_txq;         // Cola de transmisión
    QueueHandle_t SPI_rxq;         // Cola de recepción
    SemaphoreHandle_t mutex;       // Mutex para protección de acceso
    slave_t *slaves;               // Puntero a la lista de slaves
    uint8_t num_slaves;            // Número de slaves conectados
} spi__t;


