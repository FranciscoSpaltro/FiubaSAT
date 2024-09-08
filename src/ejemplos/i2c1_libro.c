#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>
#include "FreeRTOS.h"
#include "task.h"
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define I2C_SLAVE_ADDRESS 0x08 // Dirección del esclavo (Arduino)
#define systicks	xTaskGetTickCount

enum I2C_RW {
	Read = 1,
	Write = 0
};

typedef struct {
	uint32_t	device;		// I2C device
	uint32_t	timeout;	// Ticks
} I2C_Control;

static inline TickType_t
diff_ticks(TickType_t early,TickType_t later) {

	if ( later >= early )
		return later - early;
	return ~(TickType_t)0 - early + 1 + later;
}


void i2c_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // Configurar el reloj a 72MHz (blue pill)
    rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
    rcc_periph_clock_enable(RCC_I2C1); // Habilitar el reloj para el periférico I2C1
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA); // Configurar los pines GPIO PB6 y PB7 en modo de salida open-drain para el periférico I2C

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  GPIO6|GPIO7);
    gpio_set(GPIOB, GPIO6|GPIO7); // Puede no ser estrictamente necesario pero hasta que I2C esté configurado, los pines deben estar en alto
    gpio_primary_remap(0,0); // Remapear los pines I2C1 a PB6 y PB7 (PB6 y PB7 son útiles porque toleran niveles de voltaje de 5V)
}

// Configuro I2C pasandole la configuración del dispositivo, la dirección del periférico y el tiempo de espera
void i2c_configure(I2C_Control *dev, uint32_t i2c, uint32_t ticks){
    dev -> device = i2c; // Será I2C1
    dev -> timeout = ticks;

    // If I2C peripheral is stuck, it can be 
    i2c_peripheral_disable(dev -> device); // Desactivo el periférico mientras se configura
    i2c_reset(dev -> device); // Reseteo el periférico
    I2C_CR1(dev -> device) &= ~I2C_CR1_STOP; // Limpio el bit de stop

    i2c_set_standard_mode(dev -> device); // Configuro el periférico en modo estándar (100 kHz)
    i2c_set_clock_frequency(dev -> device, I2C_CR2_FREQ_36MHZ); // Configuro la frecuencia del reloj a 36 MHz
    i2c_set_trise(dev -> device, 36); // Configuro el tiempo de subida del bus (36 para 36 MHz == 1000 ns)
    i2c_set_dutycycle(dev -> device, I2C_CCR_DUTY_DIV2); // Configuro el ciclo de trabajo en 50%
    i2c_set_ccr(dev -> device, 180); // Configuro el registro de control de reloj (CCR) para 180 * 1/36M = 100 kHz
    // i2c_set_own_7bit_slave_address(dev -> device, 0x23); // Si quiero operar el controlador en modo esclavo
    i2c_peripheral_enable(dev -> device); // Habilito el periférico
}

void i2c_wait_busy(I2C_Control *dev){
    while(I2C_SR2(dev -> device) & I2C_SR2_BUSY) // Espero hasta que el bus esté ocupado
        taskYIELD();
}

void i2c_start_addr(I2C_Control *dev, uint8_t addr, enum I2C_RW rw){
    TickType_t t0 = systicks(); // Para medir el tiempo de espera

    i2c_wait_busy(dev); // Espero hasta que el bus esté ocupado
    I2C_SR1(dev -> device) &= ~I2C_SR1_AF; // Limpio la falla de ACK (si habia)
    i2c_clear_stop(dev -> device); // No genero bit de stop
    
    if(rw == Read)
        i2c_enable_ack(dev -> device); // Permite recibir el ACK del esclavo si la operación va a ser de lectura
    i2c_send_start(dev -> device); // Se solicita al periférico que genere la señal de start

    // Pruebo si el bit de start fue generado (SB)
    while(!((I2C_SR1(dev -> device) & I2C_SR1_SB) && (I2C_SR2(dev -> device) & (I2C_SR2_MSL|I2C_SR2_BUSY)))){
        if(diff_ticks(t0, systicks()) > dev -> timeout)
            exit(1);
        taskYIELD();
    }

    i2c_send_7bit_addres(dev -> device, addr, rw == Read ? I2C_READ : I2C_WRITE); // Una vez que se generó el bit de start, envío la dirección del esclavo y el indicador de lectura o escritura

    t0 = systicks(); // Reinicio el temporizador
    while(!(I2C_SR1(dev -> device) & I2C_SR1_ADDR)) { //Espero hasta que el bit ADDRESS (reconoció la dirección) esté activo, es decir, que se haya enviado la dirección del esclavo
        if(I2C_SR1(dev -> device) & I2C_SR1_AF) { // Pruebo si el esclavo respondió al request con un ACK. Si no respondió, se recibirá un NAK gracias al resistor de pull-up
            i2c_send_stop(dev -> device);
            (void) I2C_SR1(dev -> device);
            (void) I2C_SR2(dev -> device);
            exit(1);
        }
        if(diff_ticks(t0, systicks()) > dev -> timeout) {
            exit(1);
        }
        taskYIELD();
    }
    (void) I2C_SR2(dev -> device); // Limpio las flags restantes al leer el registro SR2
}

void i2c_write(I2C_Control *dev, uint8_t byte){
    TickType_t t0 = systicks();

    i2c_send_data(dev -> device, byte); // Envío el byte de datos
    while(!(I2C_SR1(dev -> device) & (I2C_SR1_BTF))){ //Espero al Byte Transfer Finished
        if(diff_ticks(t0, systicks()) > dev -> timeout)
            exit(1);
        taskYIELD();
    }
}

uint8_t i2c_read(I2C_Control *dev, bool lastf){
    TickType_t t0 = systicks();
    if(lastf) // Si es el último bit
        i2c_disable_ack(dev -> device); // Deshabilito el ACK para indicar al dispositivo I2C que no espere un ACK después de la lectura final, provocando una condición de parada automática
    while(!(I2C_SR1(dev -> device) & I2C_SR1_RxNE)){ // Espero a que el bit NOT EMPTY esté desactivado
        if(diff_ticks(t0, systicks()) > dev -> timeout)
            exit(1);
        taskYIELD();
    }

    // Una vez que el buffer no está vacío, devuelvo el byte leido
    return i2c_get_data(dev -> device);
}

void i2c_write_restart(I2C_Control *dev, uint8_t byte, uint8_t addr){
    TickType_t t0 = systicks();

    // Deshabilito las interrupciones y envío el dato junto a la conidición de inicio de la próxima comunicación antes de volver a habilitarlas
    taskENTER_CRITICAL();
    i2c_send_data(dev -> device, byte);
    i2c_send_start(dev -> device);
    taskEXIT_CRITICAL();

    // "In master mode, setting the START bit causes the interface to generate a ReStart condition at the end of the current byte transfer"
}