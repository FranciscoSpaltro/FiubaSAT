#include "FreeRTOS.h"
#include "uart.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define SIZE_BUFFER 256  // Queues size

typedef struct {
    uint32_t usart;  // USART_ID 
    QueueHandle_t txq;  // Cola de transmisión
    QueueHandle_t rxq;  // Cola de recepción donde se bufferean los datos
    SemaphoreHandle_t mutex;  // Mutex para protección de acceso
    SemaphoreHandle_t semaphore; // Semáforo para acceder a datos de rxq
    int interrupciones;  // Contador de interrupciones
} uart_t;

// Definición de estructuras UART
static uart_t uart1;
static uart_t uart2;
static uart_t uart3;

static void uart_init(uart_t *uart, uint32_t usart);
static void usart_generic_isr(uint32_t usart_id);

void UART_setup(uint32_t usart, uint32_t baudrate) {
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
        uart_init(&uart1, USART1);

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
        uart_init(&uart2, USART2);

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
        uart_init(&uart3, USART3);
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
}

static void uart_init(uart_t *uart, uint32_t usart) {
    uart->usart = usart;  // Asigna el USART correspondiente
    uart->txq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t));
    uart->rxq = xQueueCreate(SIZE_BUFFER, sizeof(uint16_t));
    uart->mutex = xSemaphoreCreateMutex();
    uart->semaphore = xSemaphoreCreateBinary();
    uart->interrupciones = 0;
}

// Manejadores de UARTs
static uart_t *get_uart(uint32_t usart_id) {
    switch (usart_id) {
        case USART1: return &uart1;
        case USART2: return &uart2;
        case USART3: return &uart3;
        default: return NULL;
    }
}

void taskUART_transmit(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uint16_t ch;
    for (;;) {
        // Intentar adquirir el mutex antes de acceder a la cola de transmisión
        if (xSemaphoreTake(uart->mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Recibir datos de la cola de transmisión
            while (xQueueReceive(uart->txq, &ch, pdMS_TO_TICKS(500)) == pdPASS) {
                // Esperar hasta que el registro de transmisión esté vacío
                while (!usart_get_flag(uart->usart, USART_SR_TXE))
                    taskYIELD(); // Ceder la CPU hasta que esté listo
                // Enviar el byte a través de USART
                usart_send_blocking(uart->usart, ch);
            }
            // Liberar el mutex después de transmitir los datos
            xSemaphoreGive(uart->mutex);
        }
        // Esperar 50 ms antes de la siguiente iteración
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

BaseType_t UART_receive(uint32_t usart_id, uint16_t *data, TickType_t xTicksToWait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return -1;

    return xQueueReceive(uart->rxq, data, xTicksToWait);
}

uint16_t UART_puts(uint32_t usart_id, const char *s) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return 0;

    uint16_t nsent = 0;
    // Recorre el string s hasta encontrar el caracter nulo
    for ( ; *s; s++) {
        // Añade el caracter a la cola uart1_txq. portMAX_DELAY indica que la tarea se bloqueará indefinidamente si la cola está llena
        if(xQueueSend(uart->txq, s, portMAX_DELAY) != pdTRUE) {
            // Si falla, se resetea la cola y se devuelve la cantidad de caracteres enviados
            xQueueReset(uart->txq);
            return nsent; // Queue full
        }
        nsent++;
    }
    return nsent;
}

void UART_putchar(uint32_t usart_id, uint16_t ch) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    xQueueSend(uart->txq, &ch, portMAX_DELAY);
}

void usart1_isr(void) {
    usart_generic_isr(USART1);
}

void usart2_isr(void) {
    usart_generic_isr(USART2);
}

void usart3_isr(void) {
    usart_generic_isr(USART3);
}

static void usart_generic_isr(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;

    uart->interrupciones++;

    // flag USART_SR_RXNE: Receive Data Register Not Empty
    while (usart_get_flag(uart->usart, USART_SR_RXNE)) {
        // Leer el byte de datos recibido del registro de datos del USART correspondiente
        uint16_t data = usart_recv_blocking(uart->usart);
        // Añade el byte de datos a la cola de recepción desde la rutina de interrupción
        if (xQueueSendToBackFromISR(uart->rxq, &data, NULL) == pdTRUE) { 
            // Dar el semáforo para indicar que hay datos disponibles en la cola
            xSemaphoreGiveFromISR(uart->semaphore, NULL);
        } else {
            // Si falla, se resetea la cola
            xQueueReset(uart->rxq);
        }
    }
}

void UART_print_buffer(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return;
    
    // Calcular el número de elementos en la cola
    UBaseType_t items_in_queue = uxQueueMessagesWaiting(uart->rxq);
    
    if (items_in_queue == 0) {
        UART_puts(USART3, "La cola está vacía.\r\n");
        return;
    }

    switch (usart_id)
    {
    case USART1:
        UART_puts(USART3, "RXQ USART 1: ");
        break;
    
    case USART2:
        UART_puts(USART3, "RXQ USART 2: ");
        break;

    case USART3:
        UART_puts(USART3, "RXQ USART 3: ");
        break;
    
    default:
        break;
    }

    uint16_t data;
    for (UBaseType_t i = 0; i < items_in_queue; i++) {
        if (xQueuePeek(uart->rxq, &data, 0) == pdTRUE) {
            UART_putchar(USART3, data);
        }
        // Sacar el siguiente elemento para avanzar en la cola
        xQueueReceive(uart->rxq, &data, 0);
        // Volver a poner el elemento para mantener la cola intacta
        xQueueSendToBack(uart->rxq, &data, 0);
    }
    UART_putchar(USART3, '\r');
    UART_putchar(USART3, '\n');
}

BaseType_t UART_semaphore_take(uint32_t usart_id, TickType_t ticks_to_wait) {
    uart_t *uart = get_uart(usart_id);
    if (uart == NULL) return pdFAIL;

    return xSemaphoreTake(uart->semaphore, ticks_to_wait);
}

void UART_semaphore_release(uint32_t usart_id) {
    uart_t *uart = get_uart(usart_id);
    if (uart != NULL) {
        xSemaphoreGive(uart->mutex);
    }
}