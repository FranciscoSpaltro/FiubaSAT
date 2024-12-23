#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"
#include "test.h"
#include "spi_driver.h"
#include "NRF24L01/nrf24l01.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "SD_CARD/fatfs_sd.h"
#include "SD_CARD/sd.h"


//#define NRF_CODE


/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

volatile uint16_t Timer1, Timer2;
TimerHandle_t xSoftTimer;
static void uart_setup(void) ;
void uart_putc(char ch);
void taskSPI1_transmit(void *pvParameterss);
void vTimerCallback(TimerHandle_t xTimer);
void sd_example(void);

// Variables para el modulo NRF
nrf24 nrfTx;
uint8_t txAddr[] = { 0xEE, 0xEE, 0xEE, 0xEE, 0xEE };

uint8_t txData[22] = "Hello From STM32";

static void taskSdWrite(void *pvParameters) {
    // Simulación de datos de telemetría
    double latitude = 45.32;
    double longitude = 122.67;
    int battery = 70;
    double temperature = 22.5;

    // Buffer para almacenar el string de telemetría
    char telemetry_buffer[128];

    // Formatear la línea de telemetría en el buffer
    sprintf(telemetry_buffer, "[TELEMETRY] COORDS: LAT=%.2f N, LON=%.2f E | BATT=%d%% | TEMP=%.1f°C\r\n",
            latitude, longitude, battery, temperature);

    // Llamar a la función para escribir en la SD
    microSD_put("Texto.txt", telemetry_buffer);

    // Eliminar la tarea al finalizar
    vTaskDelete(NULL);
}

/*static void taskSpi2(void *pvParameters) {
    spi_setup(SPI2,master_mode);
    uint8_t str[] = "Prueba de envio SPI2";

    uint16_t rxData[50] ;
    //spi_set_dff(SPI2,DATA_SIZE_16);
    for(;;){        
        spi_select_slave(SPI2,SLAVE_2);
        //spi_transmit(SPI2,str,strlen((char *)str),pdMS_TO_TICKS(10));
        spi_transmit_receive(SPI2,str,rxData,strlen((char *)str),pdMS_TO_TICKS(10));
        spi_deselect_slave(SPI2,SLAVE_2);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}*/
static void taskSpi2CRC(void *pvParameters) {
    spi_setup(SPI2,master_mode);
    
    uint8_t str[] = "Prueba de envio SPI2";

    uint16_t rxData[50] ;
    //spi_set_dff(SPI2,DATA_SIZE_16);
    for(;;){
        spi_enable_crc(SPI2);        
        spi_select_slave(SPI2,SLAVE_2);
        //spi_transmit(SPI2,str,1,pdMS_TO_TICKS(10));
        //spi_transmit_receive(SPI2,str,rxData,strlen((char *)str),pdMS_TO_TICKS(10));
        spi_set_next_tx_from_buffer(SPI2);
        spi_send(SPI2,0xFF);
        spi_read(SPI2);
        spi_set_next_tx_from_crc(SPI2);
        uint16_t crcValue = SPI_TXCRCR(SPI2);  // Obtener el CRC calculado de la recepción
        spi_xfer(SPI2,crcValue);

        spi_deselect_slave(SPI2,SLAVE_2);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void taskSdRead(void *pvParameters) {
    // Lectura del archivo texto.txt
    for (;;) {
        microSD_get("Texto.txt");
        vTaskDelay(pdMS_TO_TICKS(5000)) ;      
    }
}

/* Main loop donde arranca el programa */

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Blue Pill

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // Configuración del LED en PC13
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);

    
    xSoftTimer = xTimerCreate("SoftTimer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, vTimerCallback);
    if (xSoftTimer != NULL) {
        xTimerStart(xSoftTimer, 0);
    }

    uart_setup();       // Configuración de UART
    spi_setup(SPI1, master_mode); //Configuracion SPI
    uart_puts("Inicio el programa...\r\n");

    /*     Inicializo archivos necesarios para la SD     */
    //MX_FATFS_Init();
	//microSD_init();
	//microSD_getSize();

    //xTaskCreate(taskSdWrite, "SD WRITE", 2000, NULL, 2, NULL);  // Crear tarea para escribir memoria SD
    //xTaskCreate(taskSdRead, "SD READ", 1000, NULL, 2, NULL);  // Crear tarea para leer memoria SD
    xTaskCreate(taskBlink, "LED", 100, NULL, 2, NULL);  // Crear tarea para parpadear el LED
    //xTaskCreate(taskSpi2, "SPI2 prueba", 100, NULL, 2, NULL);  // Crear tarea para parpadear el LED
    xTaskCreate(taskSpi2CRC, "SPI2 prueba", 100, NULL, 2, NULL);  // Crear tarea para parpadear el LED

    // Inicia el scheduler de FreeRTOS
    vTaskStartScheduler();

    for (;;);
    

    #ifdef SPI1_TEST
        if (spi_setup(SPI1, master_mode) != pdTRUE){
            uart_puts("Error en spi_setup");
            return(0);
        }        // Configuración del SPI
        uart_puts("spi1_setup OK\r\n");
    #endif
   
    #ifdef NRF_CODE
        
        nrfTx.CE_port = GPIOB;
        nrfTx.CE_pin = GPIO0;
        nrfTx.CSN_port = GPIOA;
        nrfTx.CSN_pin = GPIO4;
        nrfTx.IRQ_port = NULL;
        nrfTx.IRQ_pin = NULL;
        nrfTx.SPI_id = SPI1;

        
        nrf24_init(&nrfTx);
        nrf24_setTxAddr(&nrfTx, txAddr);
        //checkNRFConfig(&nrfTx);
        uint8_t data = 'H';
        while(1){       	       

            #ifdef NRF_TEST           
            nrf24_setMode(&nrfTx, txMode);

            if (nrf24_Transmit(&nrfTx, txData, sizeof(txData)) == 1) {
                gpio_toggle(GPIOC, GPIO13);
                nrf24_setMode(&nrfTx, standby);
            }

            for (int i = 0; i < 7200000; i++)	
                __asm__("nop");
            #endif

            spi_select_slave(SPI1, SLAVE_1);
            
            spi_transmit(SPI1, &data, 1,10);

            spi_deselect_slave(SPI1, SLAVE_1);

            for (int i = 0; i < 7200000; i++)	
                __asm__("nop");

            gpio_toggle(GPIOC,GPIO13);

            for (int i = 0; i < 7200000; i++)	
                __asm__("nop");

        }
    
    #endif

    #ifdef SPI2_TEST
        uint16_t data = 0xABCD;
        
        while(1){       	       

                spi_select_slave(SPI2, SLAVE_2);                
                
                spi_xfer(SPI2,data);
                
                spi_deselect_slave(SPI2, SLAVE_2);
                

                for (int i = 0; i < 7200000; i++)	
                    __asm__("nop");

                gpio_toggle(GPIOC,GPIO13);

                for (int i = 0; i < 7200000; i++)	
                    __asm__("nop");

        
            }
    #endif
    
  
    return 0;
}





static void uart_setup(void) {

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // UART TX on PA9 (GPIO_USART1_TX)
    gpio_set_mode(GPIOA,
                  GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_USART1_TX);

    usart_set_baudrate(USART1, 38400);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

void uart_putc(char ch) {
    usart_send_blocking(USART1, ch); // Envía un solo carácter por UART
}
/*
void uart_puts(const char *str) {
    while (*str) {
        uart_putc(*str); // Envía el carácter actual
        str++;           // Avanza al siguiente carácter en la cadena
    }
}*/

// Asumiendo que uart_txq y usart_get_flag/usart_send están definidos e inicializados

void usart_Transmit(QueueHandle_t uart_txq) {
    char ch;

    for (;;) {
        // Recibir carácter para transmitir
        if (xQueueReceive(uart_txq, &ch, 500) == pdPASS) {
            // Esperar hasta que el registro de transmisión esté vacío
            while (!usart_get_flag(USART1, USART_SR_TXE)) {
                taskYIELD(); // Ceder tiempo de CPU hasta que esté listo
            }
            // Enviar carácter
            usart_send(USART1, ch);
        }
    }
}

void vTimerCallback(TimerHandle_t xTimer) {
    if (Timer1 > 0) Timer1--; // Decrementa Timer1
    if (Timer2 > 0) Timer2--; // Decrementa Timer2        
    
}

void uart_puts(const char *str) {
    while (*str) {
        uart_putc(*str); // Envía el carácter actual
        str++;           // Avanza al siguiente carácter en la cadena
    }
}