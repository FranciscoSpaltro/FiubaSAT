#include "FreeRTOS.h"

#include "uart.h"
#include "circular_buffer.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SIZE_BUFFER 512  // Buffer sizes

/**
 * @brief Estructura para manejar la configuración de UART.
 * 
 * Esta estructura contiene información necesaria para manejar las UART, incluyendo 
 * estructuras de datos para la transmisión y recepción, mutexes para protección de
 * acceso, y un contador de interrupciones.
 */
typedef struct {
    uint32_t usart;  // Identificador del USART.
    QueueHandle_t txq;  // Cola para la transmisión de datos.
    circular_buffer_t *rxq;  // Buffer circular de recepción para almacenar los datos recibidos.
    SemaphoreHandle_t mutex;  // Mutex para proteger el acceso a los recursos compartidos.
    SemaphoreHandle_t semaphore; // Semáforo para señalizar la disponibilidad de datos en rxq.
    int interrupciones;  // Contador de interrupciones.
} uart_t;

// Definición de estructuras UART
static uart_t uart1;
static uart_t uart2;
static uart_t uart3;

// Prototipos de funciones
static BaseType_t uart_init(uart_t *uart, uint32_t usart);
static void usart_generic_isr(uint32_t usart_id);

/**
 * @brief Obtiene el manejador de UART basado en el identificador del USART.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @return puntero a la estructura uart_t correspondiente, o NULL si el identificador no es válido.
 */
static uart_t *get_uart(uint32_t usart_id) {
    switch (usart_id) {
        case USART1: return &uart1;
        case USART2: return &uart2;
        case USART3: return &uart3;
        default: return NULL;
    }
}

/**
 * @brief Configura el USART especificado con la velocidad de baudios dada.
 * 
 * @param usart Identificador del USART (USART1, USART2, USART3).
 * @param baudrate Velocidad de baudios para la configuración del USART.
 * @return pdPASS si la configuración fue exitosa, pdFAIL en caso contrario.
 */
BaseType_t UART_setup(uint32_t usart, uint32_t baudrate) {
    uart_t *uart = get_uart(usart);
    if (uart == NULL) return pdFAIL;

    // Configuración del reloj y pines según el USART
    if (usart == USART1) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART1)
        rcc_periph_clock_enable(RCC_GPIOA); 
        // Habilitar el clock para USART1
        rcc_periph_clock_enable(RCC_USART1); 

        // Configurar los pines de UART1 (PA9 TX y PA10 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART1_TX, 
            GPIO_MODE_OUTPUT_50_MHZ,
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART1_TX);

        gpio_set_mode(GPIO_BANK_USART1_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART1_RX);
        
        // Habilitar la interrupción de UART1 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART1_IRQ);
        if(uart_init(&uart1, USART1) != pdPASS) return pdFAIL;

    } else if (usart == USART2) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART2)
        rcc_periph_clock_enable(RCC_GPIOA);
        // Habilitar el clock para USART2
        rcc_periph_clock_enable(RCC_USART2);

        // Configurar los pines de UART2 (PA2 TX y PA3 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART2_TX, 
            GPIO_MODE_OUTPUT_50_MHZ, 
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART2_TX);

        gpio_set_mode(GPIO_BANK_USART2_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART2_RX);

        // Habilitar la interrupción de UART2 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART2_IRQ);
        if(uart_init(&uart2, USART2) != pdPASS) return pdFAIL;

    } else if (usart == USART3) {
        // Habilitar el clock para GPIOA (donde están conectados los pines TX y RX de UART3)
        rcc_periph_clock_enable(RCC_GPIOB);
        // Habilitar el clock para USART3
        rcc_periph_clock_enable(RCC_USART3);

        // Configurar los pines de UART1 (PB10 TX y PB11 RX)
        // gpio_set_mode(puerto GPIO afectado, input/output y velocidad de cambio, modo de salida, pin afectado)
        gpio_set_mode(GPIO_BANK_USART3_TX, 
            GPIO_MODE_OUTPUT_50_MHZ, 
            GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
            GPIO_USART3_TX);

        gpio_set_mode(GPIO_BANK_USART3_RX, 
            GPIO_MODE_INPUT, 
            GPIO_CNF_INPUT_FLOAT, 
            GPIO_USART3_RX);
        
        // Habilitar la interrupción de UART3 en el NVIC (a nivel sistema para que el controlador de interrupciones pueda manejarla)
        nvic_enable_irq(NVIC_USART3_IRQ);
        if(uart_init(&uart3, USART3) != pdPASS) return pdFAIL;
    }

    // Configuración de USART 
    usart_set_baudrate(usart, baudrate);
    usart_set_databits(usart, 8);
    usart_set_stopbits(usart, USART_STOPBITS_1);
    usart_set_mode(usart, USART_MODE_TX_RX);
    usart_set_parity(usart, USART_PARITY_NONE);
    usart_set_flow_control(usart, USART_FLOWCONTROL_NONE);

    usart_enable(usart);
    // Dentro de las fuentes de interrupción de UART, habilitar la interrupción de recepción
    usart_enable_rx_interrupt(usart);

    return pdPASS;
}

/**
 * @brief Inicializa la estructura de UART.
 * 
 * Configura la cola de transmisión, la cola de recepción, y los semáforos para la UART
 * especificada.
 * 
 * @param uart Puntero a la estructura uart_t que se desea inicializar.
 * @param usart Identificador del USART (USART1, USART2, USART3).
 * @return pdPASS si la inicialización fue exitosa, pdFAIL en caso contrario.
 */
static BaseType_t uart_init(uart_t *uart, uint32_t usart) {
    uart->usart = usart;  // Asigna el USART correspondiente
    uart->txq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t)); // Crea la cola de transmisión
    if (uart->txq == NULL) return pdFAIL;

    uart->rxq = xCircularBufferCreate(SIZE_BUFFER); // Crea la cola de recepción
    if (uart->rxq == NULL) {
        vQueueDelete(uart->txq);
        return pdFAIL;
    }

    uart->mutex = xSemaphoreCreateMutex();
    if (uart->mutex == NULL) {
        vQueueDelete(uart->txq);
        vCircularBufferDelete(uart->rxq);
        return pdFAIL;
    }
    xSemaphoreGive(uart->mutex);

    uart->semaphore = xSemaphoreCreateBinary();
    if (uart->semaphore == NULL) {
        vQueueDelete(uart->txq);
        vCircularBufferDelete(uart->rxq);
        vSemaphoreDelete(uart->mutex);
        return pdFAIL;
    }

    uart->interrupciones = 0;
    return pdPASS;
}

/**
 * @brief Tarea para la transmisión de datos a través de UART.
 * 
 * Lee los datos de la cola de transmisión y los envía a través del USART especificado.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 */
void taskUART_transmit(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uint8_t ch;
    for (;;) {
            while (xQueueReceive(uart->txq, &ch, pdMS_TO_TICKS(500)) == pdPASS) {
                // Esperar hasta que el registro de transmisión esté vacío
                while (!usart_get_flag(uart->usart, USART_SR_TXE))
                    taskYIELD(); // Ceder la CPU hasta que esté listo
                // Enviar el byte a través de USART
                usart_send(uart->usart, (uint8_t)ch);
            }
        // Esperar 50 ms antes de la siguiente iteración
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/**
 * @brief Recibe datos de UART y los almacena en el buffer de receipción.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param data Puntero a la variable donde se almacenará el dato recibido.
 * @return pdTRUE si el dato fue recibido exitosamente, pdFAIL en caso contrario.
 */
BaseType_t UART_receive(uint32_t usart_id, uint8_t *data) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    return pop(uart->rxq, data);
}

/**
 * @brief Limpia la cola de recepción de UART.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param xTicksToWait Tiempo de espera para tomar el mutex.
 * @return pdPASS si la limpieza fue exitosa, pdFAIL en caso contrario.
 */
BaseType_t UART_clear_rx_queue(uint32_t usart_id, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    // Aquí se usa un mutex para proteger el acceso a la cola
    if(xSemaphoreTake(uart->mutex, xTicksToWait) != pdPASS) return pdFAIL;

    circular_buffer_reset(uart->rxq);

    if(xSemaphoreGive(uart->mutex) != pdTRUE) return pdFAIL;
    
    return pdPASS;
}

/**
 * @brief Envía una cadena de caracteres a través de UART.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param s Cadena de caracteres a enviar.
 * @param xTicksToWait Tiempo de espera para enviar cada carácter.
 * @return Número de caracteres enviados.
 */
uint16_t UART_puts(uint32_t usart_id, const char *s, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return 0;

    uint16_t nsent = 0;
    // Recorre el string s hasta encontrar el caracter nulo
    for ( ; *s; s++) {
        // Añade el caracter a la cola uart1_txq. Espera xTicksToWait (portMAX_DELAY espera indefinidamente)
        if(xQueueSend(uart->txq, s, xTicksToWait) != pdTRUE) {
            return nsent; // Queue full nsent < strlen(s)
        }
        nsent++;
    }
    return nsent;
}

/**
 * @brief Envía un carácter a través de UART.
 * 
 * Coloca el carácter en la cola de transmisión de la UART especificada y espera 
 * a obtener el mutex para garantizar el acceso exclusivo.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param ch Carácter a enviar.
 * @param xTicksToWait Tiempo de espera para obtener el mutex.
 * @return pdPASS si el carácter se envió exitosamente, pdFAIL en caso contrario.
 */
BaseType_t UART_putchar(uint32_t usart_id, uint8_t ch, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;
    
    if (xSemaphoreTake(uart->mutex, xTicksToWait) != pdTRUE) return pdFAIL;
    
    if (xQueueSend(uart->txq, &ch, xTicksToWait) != pdTRUE) {
        xSemaphoreGive(uart->mutex);
        return pdFAIL;
    }

    xSemaphoreGive(uart->mutex);
    return pdPASS;
}

/**
 * @brief Imprime el contenido de la cola de recepción de UART en la UART especificada.
 * 
 * Lee los datos de la cola de recepción y los envía a través de USART3 para su visualización.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 */
void UART_print_buffer(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;
    
    // Calcular el número de elementos en la cola
    uint16_t items_in_queue = serial_available(uart->rxq);

    if (items_in_queue == 0) {
        switch (usart_id)
        {
        case USART1:
            UART_puts(USART3, "USART 1: ", pdMS_TO_TICKS(500));
            break;
        
        case USART2:
            UART_puts(USART3, "USART 2: ", pdMS_TO_TICKS(500));
            break;

        case USART3:
            UART_puts(USART3, "USART 3: ", pdMS_TO_TICKS(500));
            break;
        
        default:
            break;
        }

        UART_puts(USART3, "La cola está vacía.\r\n", pdMS_TO_TICKS(500));
        return;
    }

    switch (usart_id)
    {
    case USART1:
        UART_puts(USART3, "RXQ USART 1: ", pdMS_TO_TICKS(500));
        break;
    
    case USART2:
        UART_puts(USART3, "RXQ USART 2: ", pdMS_TO_TICKS(500));
        break;

    case USART3:
        UART_puts(USART3, "RXQ USART 3: ", pdMS_TO_TICKS(500));
        break;
    
    default:
        break;
    }

    uint8_t data;
    for (uint16_t i = 0; i < items_in_queue; i++) {
        if (pop(uart->rxq, &data) == pdPASS) {
            UART_putchar(USART3, data, pdMS_TO_TICKS(500));
            push(uart->rxq, data);
        }
    }
    UART_putchar(USART3, '\r', pdMS_TO_TICKS(500));
    UART_putchar(USART3, '\n', pdMS_TO_TICKS(500));
}

/**
 * @brief Toma el semáforo de UART.
 * 
 * Intenta tomar el semáforo de la UART especificada para asegurar el acceso 
 * exclusivo a los recursos.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @param ticks_to_wait Tiempo de espera para tomar el semáforo.
 * @return pdTRUE si se tomó el semáforo exitosamente, pdFALSE en caso contrario.
 */
BaseType_t UART_semaphore_take(uint32_t usart_id, TickType_t ticks_to_wait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    return xSemaphoreTake(uart->semaphore, ticks_to_wait);
}

/**
 * @brief Libera el semáforo de UART.
 * 
 * Libera el semáforo de la UART especificada para permitir el acceso a otros recursos.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 * @return pdPASS si se liberó el semáforo exitosamente, pdFAIL en caso contrario.
 */
BaseType_t UART_semaphore_release(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;
    
    return xSemaphoreGive(uart->semaphore);
}

/**
 * @brief Maneja las interrupciones de USART1.
 * 
 * Llama a la función de manejo genérico de interrupciones para USART1.
 */
void usart1_isr(void) {
    usart_generic_isr(USART1);
}

/**
 * @brief Maneja las interrupciones de USART2.
 * 
 * Llama a la función de manejo genérico de interrupciones para USART2.
 */
void usart2_isr(void) {
    usart_generic_isr(USART2);
}

/**
 * @brief Maneja las interrupciones de USART3.
 * 
 * Llama a la función de manejo genérico de interrupciones para USART3.
 */
void usart3_isr(void) {
    usart_generic_isr(USART3);
}

/**
 * @brief Maneja las interrupciones genéricas de USART.
 * 
 * Se encarga de recibir datos y almacenarlos en la cola de recepción correspondiente
 * según el identificador del USART.
 * 
 * @param usart_id Identificador del USART (USART1, USART2, USART3).
 */
static void usart_generic_isr(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    if (usart_get_flag(USART1, USART_SR_RXNE)) {
        uint8_t ndata = (uint8_t)usart_recv(USART1);
        push(uart->rxq, ndata);
    }

    if (usart_get_flag(USART2, USART_SR_RXNE)) {
        uint8_t ndata = (uint8_t)usart_recv(USART2);
        push(uart->rxq, ndata);
    }

    if (usart_get_flag(USART3, USART_SR_RXNE)) {
        uint8_t ndata = (uint8_t)usart_recv(USART3);
        push(uart->rxq, ndata);
    }
}

/*
void usart1_isr(void) {
    usart_generic_isr(USART1);
}

void usart2_isr(void) {
    usart_generic_isr(USART2);
}

void usart3_isr(void) {
    usart_generic_isr(USART3);
}

// Rutina de interrupción genérica para USART
static void usart_generic_isr(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    // Incrementar el contador de interrupciones (auxiliar)
    uart->interrupciones++;

    // flag USART_SR_RXNE: Receive Data Register Not Empty
    while (usart_get_flag(uart->usart, USART_SR_RXNE)) {
        // Leer el byte de datos recibido del registro de datos del USART correspondiente
        uint16_t data = usart_recv_blocking(uart->usart);

        // Verificar si la cola tiene espacio disponible antes de encolar
        if (uxQueueSpacesAvailable(uart->rxq) > 0) {
            // Añadir el byte de datos a la cola de recepción desde la rutina de interrupción
            if (xQueueSendToBackFromISR(uart->rxq, &data, NULL) == pdTRUE) { 
                // Dar el semáforo para indicar que hay datos disponibles en la cola
                xSemaphoreGiveFromISR(uart->semaphore, NULL);
            } else {
                UART_puts(USART3, "Error al encolar datos en la cola de recepción.\r\n", pdMS_TO_TICKS(100));
            }
        } else {
            // Manejar el caso donde la cola está llena
            UART_puts(USART3, "Cola de recepción llena, no se pueden encolar más datos.\r\n", pdMS_TO_TICKS(100));
        }
    }
}

*/

uint16_t get_rxq_buffer(uint32_t usart_id, char *output, uint16_t max_len) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) {
        return 0;
    }

    return circular_buffer_to_string(uart->rxq, output, max_len);
}

uint16_t UART_available_data(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) {
        return 0;
    }

    return serial_available(uart->rxq);
}