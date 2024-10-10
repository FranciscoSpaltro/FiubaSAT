#include "FreeRTOS.h"
#include "task.h"
#include "uart.h"

#include "blink.h"
#include "timers.h"
#include <stdio.h>
#include "semphr.h"
#include "test.h"
#include "spi_driver.h"
#include "nrf24l01.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>


//#define NRF_CODE


/**
 * @brief SPI2 Interrupt service routine.
 */
void spi2_isr(void)
{   
    uart_puts("Entre a la interrupcion\r\n");
    /* Wait for 'Busy' flag to reset. */
    while ((SPI_SR(SPI2) & SPI_SR_BSY))
    {
    }
    
    uint8_t indata = spi_read(SPI2);
    uart_putc(indata);
    
    /* Clear 'Read data register not empty' flag. */
    SPI_SR(SPI2) &= ~SPI_SR_RXNE;
}

/* Handler en caso de que la aplicación cause un overflow del stack */
void vApplicationStackOverflowHook(TaskHandle_t xTask __attribute__((unused)), char *pcTaskName __attribute__((unused))) {
	for (;;);
}

static void delay(volatile uint32_t count) {
    while (count--) {
        __asm__("NOP");
    }
}

static void uart_setup(void) ;
void uart_putc(char ch);
void taskSPI1_transmit(void *pvParameterss);
void systick_setup();

// Variables para el modulo NRF
nrf24 nrfTx;
uint8_t txAddr[] = { 0xEE, 0xEE, 0xEE, 0xEE, 0xEE };

uint8_t txData[22] = "Hello From STM32";

/* Main loop donde arranca el programa */

int main(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Blue Pill

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    // Configuración del LED en PC13
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);
    
    uart_setup();       // Configuración de UART

    #ifdef SPI1_TEST
        if (spi_setup(SPI1, master_mode) != pdTRUE){
            uart_puts("Error en spi_setup");
            return(0);
        }        // Configuración del SPI
        uart_puts("spi1_setup OK\r\n");
    #endif

    
    if (spi_setup(SPI2, master_mode) != pdTRUE){
        uart_puts("Error en spi2_setup");
        return(0);
    }        // Configuración del SPI
    //spi_set_dff(SPI2,DATA_SIZE_16);
    //spi_send_lsb_first(SPI2);
    uart_puts("spi2_setup OK\r\n");
    
    //--------------------------------------- MODULE SETTINGS NRF ----------------------------------------------
	
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

    //systick_setup();    
  
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

void uart_puts(const char *str) {
    while (*str) {
        uart_putc(*str); // Envía el carácter actual
        str++;           // Avanza al siguiente carácter en la cadena
    }
}

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


/*

void spi2_send_data(uint8_t data) {
    // Activar NSS manualmente (bajar el pin NSS) 
    gpio_clear(GPIOA, GPIO4);  // Poner NSS en bajo (activo)

    // Enviar el dato por SPI 
    spi_send(SPI1, data);

    // Esperar hasta que se complete la transmisión 
    while (!(SPI_SR(SPI1) & SPI_SR_TXE));  // Esperar a que el buffer de transmisión esté vacío
    while (SPI_SR(SPI1) & SPI_SR_BSY);     // Esperar a que el bus SPI no esté ocupado

    // Desactivar NSS manualmente (subir el pin NSS) 
    gpio_set(GPIOA, GPIO4);  // Poner NSS en alto (inactivo)
}



int main(void) {
    // Configurar SPI 
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOC);

    // Configuración del LED en PC13
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
    spi_setup(SPI1);

    uint8_t txData[22] = "Hello from STM32 \n";
    uint8_t *ptr = txData;  // Inicializar puntero que recorrerá la cadena

    // Bucle principal 
    while (1) {
        // Enviar un dato por SPI 
        spi_send_data(*ptr++);  // Enviar el carácter actual y avanzar el puntero

        if (*ptr == '\0') {     // Si llega al final de la cadena
            ptr = txData;       // Reiniciar el puntero al inicio
            gpio_toggle(GPIOC, GPIO13);  // Alternar el estado del LED
        }

        // Pequeña espera 
        for (int i = 0; i < 100000; i++) { __asm__("nop"); }
    }

    return 0;
}

*/

void systick_setup(){
    /*** Configuración del SysTyck ***/
    // Se toma la velocidad del AHB (SYSCLK) y se activa el divisor por 8:
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); // 72M/8 = 9M

    // Se carga el valor a cargar cuando el SysTick llega a 0 (downflow):
    systick_set_reload(8999); // 9M / 9000 = 1k => T = 1ms
    
    // Se habilita la interrupción del SysTick:
    systick_interrupt_enable();
    
    // El SysTick empieza a contar:
    systick_counter_enable();

}