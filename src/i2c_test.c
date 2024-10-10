#include "i2c_test.h"

/**
 * @brief Tarea de testing que envia un mensaje periódico al 0x08 por I2C, testeando el bus
 * 
 * @param pvParameters Sin utilizar
 * @return void
 */

void test_i2c(void *pvParameters) {
    uint32_t i2c_id = I2C1;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C. Eliminando tarea...\n\r");
        vTaskDelete(NULL);
    }

    for (;;) {
        if (print_i2c("Hola, Arduino!", 0x08) != pdPASS) {
            print_uart("Error: No se pudo enviar el mensaje (UNO).\n\r");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }


        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * @brief Tarea de testing para solicitar la temperatura y ebviarla al Arduino
 *       Solicita tres bytes al HTU21D, calcula la temperatura y la envía al Arduino, todo por I2C
 * 
 * @param pvParameters Sin utilizar
 * @return void
 */

void i2c_testing_trama(void *pvParameters) {
    uint32_t i2c_id = I2C1;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C. Eliminando tarea...\n\r");
        vTaskDelete(NULL);
    }
    
    bool reset = false;

    for (;;) {
        print_uart("---------------Comienza-----------\n\r");
        uint8_t data[3];
        char data_str[20]; // Asegúrate de que el tamaño sea suficiente para la cadena
        
        if(reset_htu21d(i2c_id) != pdPASS){
            print_uart("Error: No se pudo realizar el reset del sensor.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (request_htu21d(i2c_id, TRIGGER_TEMP_MEASURE_NOHOLD) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de temperatura.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(100));


        // Desencolo response
        uint8_t data_aux;
        bool error = false;

        for(int i = 0; i < 3; i++){
            if(dequeue_i2c_response(i2c->responses, &data_aux) != I2C_PASS){
                print_uart("Error: No se pudo recibir el mensaje de temperatura.");
                error = true;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            data[i] = data_aux;
        }

        if(error){
            continue;
        }

        uint16_t rawTemperature = (data[0] << 8) | data[1];
        rawTemperature &= 0xFFFC;

        float temp = -46.85 + (175.72 * rawTemperature / 65536.0);  // Convertimos los datos a temperatura en grados Celsius
   
        snprintf(data_str, sizeof(data_str), "Temp: %.2f °C", temp); // Formatea la temperatura a dos decimales como cadena
    
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != pdPASS){
            print_uart("Error: No se pudo enviar el mensaje data_str.\r\n");
            reset = true;
            continue;
        }
        
        vTaskDelay(pdMS_TO_TICKS(2500));


        memset(data_str, 0, sizeof(data_str));
    }
}

/**
 * @brief Tarea de testing para solicitar la temperatura y ebviarla al Arduino
 *       Solicita tres bytes al HTU21D, calcula la temperatura y la envía al Arduino, todo por I2C
 * 
 * @param pvParameters Sin utilizar
 * @return void
 */

void test_request_i2c(void *pvParameters) {
    uint32_t i2c_id = I2C1;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C. Eliminando tarea...\n\r");
        vTaskDelete(NULL);
    }
    
    bool reset = false;

    for (;;) {
        print_uart("---------------Comienza-----------\n\r");
        uint8_t data[3];
        char data_str[20]; // Asegúrate de que el tamaño sea suficiente para la cadena
        
        if(reset){
            //i2c_reset_bus(i2c_id);
            print_uart("Eliminando tarea...\n\r");
            vTaskDelete(NULL);
            reset = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        print_uart("DEBUG1\n\r");
        if(reset_htu21d(i2c_id) != pdPASS){
            print_uart("Error: No se pudo realizar el reset del sensor.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        print_uart("DEBUG2\n\r");
        if (request_htu21d(i2c_id, TRIGGER_TEMP_MEASURE_NOHOLD) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de temperatura.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));


        // Desencolo response
        uint8_t data_aux;
        bool error = false;

        for(int i = 0; i < 3; i++){
            if(dequeue_i2c_response(i2c->responses, &data_aux) != I2C_PASS){
                print_uart("Error: No se pudo recibir el mensaje de temperatura.");
                error = true;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;
            }
            data[i] = data_aux;
        }

        if(error){
            reset = true;
            continue;
        }

        uint16_t rawTemperature = (data[0] << 8) | data[1];
        rawTemperature &= 0xFFFC;

        float temp = -46.85 + (175.72 * rawTemperature / 65536.0);  // Convertimos los datos a temperatura en grados Celsius
   
        snprintf(data_str, sizeof(data_str), "Temp: %.2f °C", temp); // Formatea la temperatura a dos decimales como cadena
    
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != pdPASS){
            print_uart("Error: No se pudo enviar el mensaje data_str.\r\n");
            reset = true;
            continue;
        }
        
        vTaskDelay(pdMS_TO_TICKS(2500));


        memset(data_str, 0, sizeof(data_str));

        if (request_htu21d(i2c_id, TRIGGER_HUMD_MEASURE_NOHOLD) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de humedad.");
            reset = true;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));


        // Desencolo response
        for(int i = 0; i < 3; i++){
            if(dequeue_i2c_response(i2c->responses, &data_aux) != I2C_PASS){
                print_uart("Error: No se pudo recibir el mensaje de humedad.");
                error = true;
                break;
            }
            data[i] = data_aux;
        }

        if(error){
            reset = true;
            continue;
        }

        uint16_t rawHumidity = (data[0] << 8) | data[1];  // Leemos los dos primeros bytes

        rawHumidity &= 0xFFFC;  // Aplicamos máscara para quitar los bits de estado
        float hum = -6.0 + (125.0 * rawHumidity / 65536.0);  // Convertimos los datos a porcentaje de humedad

        snprintf(data_str, sizeof(data_str), "Hum: %.2f %%", hum); // Formatea la temperatura a dos decimales como cadena
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != pdPASS){
            print_uart("Error: No se pudo enviar el mensaje hum_str.");
            reset = true;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));

        print_uart("---------------\n\r");
    }
}

/**
 * @brief Imprime un mensaje por UART
 * 
 * @param s Mensaje a imprimir
 * @return void
 */
void print_uart(const char *s){
    UART_puts(USART1, s, pdMS_TO_TICKS(500));
}