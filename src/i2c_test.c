#include "i2c_test.h"

uint8_t htu21d_data[3];

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
        if (print_i2c("Hola, Arduino!", 0x08) != I2C_PASS) {
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

void test_request_i2c(void *pvParameters) {
    uint32_t i2c_id = I2C1;

    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C. Eliminando tarea...\n\r");
        vTaskDelete(NULL);
    }
    
    bool reset = false;
    char data_str[20];

    for (;;) {
        if(reset){
            print_uart("Eliminando tarea...\n\r");
            vTaskDelete(NULL);
            reset = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        if(reset_htu21d(i2c_id) != pdPASS){
            print_uart("Error: No se pudo realizar el reset del sensor.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (request_htu21d(i2c_id, TRIGGER_TEMP_MEASURE_NOHOLD, htu21d_data) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de temperatura.\n\r");
            reset = true;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));

        uint16_t rawTemperature = (htu21d_data[0] << 8) | htu21d_data[1];
        rawTemperature &= 0xFFFC;

        float temp = -46.85 + (175.72 * rawTemperature / 65536.0);  // Convertimos los datos a temperatura en grados Celsius
   
        snprintf(data_str, sizeof(data_str), "Temp: %.2f °C", temp); // Formatea la temperatura a dos decimales como cadena
    
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != I2C_PASS){
            print_uart("Error: No se pudo enviar el mensaje data_str.\r\n");
            reset = true;
            continue;
        }
        
        vTaskDelay(pdMS_TO_TICKS(2500));


        memset(data_str, 0, sizeof(data_str));

        if (request_htu21d(i2c_id, TRIGGER_HUMD_MEASURE_NOHOLD, htu21d_data) != pdPASS) {
            print_uart("Error: No se pudo realizar la solicitud de humedad.");
            reset = true;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));


        uint16_t rawHumidity = (htu21d_data[0] << 8) | htu21d_data[1];  // Leemos los dos primeros bytes

        rawHumidity &= 0xFFFC;  // Aplicamos máscara para quitar los bits de estado
        float hum = -6.0 + (125.0 * rawHumidity / 65536.0);  // Convertimos los datos a porcentaje de humedad

        snprintf(data_str, sizeof(data_str), "Hum: %.2f %%", hum); // Formatea la temperatura a dos decimales como cadena
        if(print_i2c(data_str, I2C_ARDUINO_ADDRESS) != I2C_PASS){
            print_uart("Error: No se pudo enviar el mensaje hum_str.");
            reset = true;
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(2500));

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