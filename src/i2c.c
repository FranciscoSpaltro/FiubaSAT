#include "i2c.h"

SemaphoreHandle_t i2c_mutex;

void i2c_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOB); // Habilitar el reloj para el puerto B
    rcc_periph_clock_enable(RCC_I2C1); // Habilitar el reloj para I2C1

    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO_I2C1_SCL | GPIO_I2C1_SDA); // Configurar pines para I2C

    i2c_peripheral_disable(I2C1); // Desactivar I2C1 antes de configurar
    rcc_periph_reset_pulse(RST_I2C1);

    i2c_set_standard_mode(I2C1); // Configurar modo estándar
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ); // Configurar frecuencia del reloj a 36 MHz
    i2c_set_trise(I2C1, 36); // Configurar tiempo de subida
    i2c_set_dutycycle(I2C1, I2C_CCR_DUTY_DIV2); // Configurar ciclo de trabajo
    i2c_set_ccr(I2C1, 180); // Configurar CCR para 100 kHz
    i2c_peripheral_enable(I2C1); // Habilitar I2C1

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        print_uart("Error al crear el semáforo\n\r");
        while(1);
    }
}

void i2c_wait_until_ready(void) {
    while (I2C_SR2(I2C1) & I2C_SR2_BUSY) {
        taskYIELD();
    }
}

void i2c_start(uint8_t addr, bool read) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_wait_until_ready();
    i2c_send_start(I2C1);
    while (!(I2C_SR1(I2C1) & I2C_SR1_SB)) {
        // Esperar hasta que el bit de Start esté establecido
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    i2c_send_7bit_address(I2C1, addr, read ? I2C_READ : I2C_WRITE);
    while (!(I2C_SR1(I2C1) & I2C_SR1_ADDR)) {
        // Esperar hasta que el bit de Address esté establecido
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Limpiar la bandera de dirección
    (void)I2C_SR2(I2C1);
    xSemaphoreGive(i2c_mutex);
    
}

void i2c_write(uint8_t data) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_send_data(I2C1, data);
    // Esperar hasta que se complete la transferencia de datos
    while (!(I2C_SR1(I2C1) & I2C_SR1_BTF)) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Espera de 10 ms
    }

    // Comprobar el bit de ACK después de la transferencia
    if (I2C_SR1(I2C1) & I2C_SR1_AF) {
        // No se recibió ACK del esclavo (NACK)
        // Limpia la bandera de fallo de ACK
        I2C_SR1(I2C1) &= ~I2C_SR1_AF;
        // Manejar el error de NACK aquí (por ejemplo, reenviar el mensaje o detener la comunicación)
        print_uart("No se recibio ACK\n\r");
    } else {
        // ACK recibido correctamente
        print_uart("Se recibio ACK\n\r");
    }
    xSemaphoreGive(i2c_mutex);
}


uint8_t i2c_read(bool last) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    if (last) {
        i2c_disable_ack(I2C1);
    } else {
        i2c_enable_ack(I2C1);
    }

    while (!(I2C_SR1(I2C1) & I2C_SR1_RxNE)) {
        // Esperar hasta que el buffer de recepción no esté vacío
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    xSemaphoreGive(i2c_mutex);
    return i2c_get_data(I2C1);
}

void i2c_stop(void) {
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_send_stop(I2C1);
    xSemaphoreGive(i2c_mutex);
}

void task_i2c(void *pvParameters) {
    uint8_t respuesta;
    char buffer[50]; // Ajusta el tamaño según sea necesario
    int mensaje = 0;
    
    for(;;) {
        i2c_start(I2C_SLAVE_ADDRESS, false); // Iniciar comunicación con el Arduino
        i2c_write(mensaje);
        i2c_stop();


        vTaskDelay(pdMS_TO_TICKS(100)); // Esperar a que el Arduino procese el comando

        i2c_start(I2C_SLAVE_ADDRESS, true); // Leer desde el Arduino
        respuesta = i2c_read(true); // Leer la hora (último byte)
        i2c_stop();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo antes de la próxima solicitud
    }
}














/******************************
 * TESTING
 * ***************************/
 
 void print_uart(const char *s){
    UART_puts(USART1, s, pdMS_TO_TICKS(500));
}