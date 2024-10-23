void simply_exti0_isr(void) {
    if (exti_get_flag_status(EXTI0)) {
        timer_set_counter(TIM2, 0);
        timer_clear_flag(TIM2, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
        timer_enable_counter(TIM2);

        start_time = xTaskGetTickCount(); // Capturar el tiempo de inicio

        // Limpiar la bandera de interrupción externa
        exti_reset_request(EXTI0);
    }
}

void simply_tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        timer_disable_counter(TIM2);  // Detener el temporizador

        //timer_clear_flag(TIM2, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
        //timer_clear_flag(TIM2, TIM_SR_UIF); // Limpiar la bandera del temporizador
        

        end_time = xTaskGetTickCount(); // Capturar el tiempo final

        uint32_t elapsed_time = end_time - start_time;
        float tiempo_transcurrido = (float)elapsed_time / pdMS_TO_TICKS(1000);

        // Mostrar el resultado
        char aux[50];
        snprintf(aux, sizeof(aux), "Tiempo pasado: %.2f ms\n\r", tiempo_transcurrido);
        print_uart(aux);
        print_uart("--------------------\n\r");
    }
}

void vLifeLineTask(void *pvParameters) {
    // No es necesario verificar GPIO en este caso, asumiendo que está configurado correctamente
    for (;;) {
        gpio_toggle(TEST_PORT, LIFE_LINE_PIN); // Alterna el estado del pin
        vTaskDelay(pdMS_TO_TICKS(LIFE_LINE_DELAY)); // Espera LIFE_LINE_DELAY segundos
    }
}

/**
 * @brief Tarea de testing para solicitar la temperatura y enviarla al Arduino
 *       Solicita tres bytes al HTU21D, calcula la temperatura y la envía al Arduino, todo por I2C
 * 
 * @param pvParameters Sin utilizar
 * @return void
 */

void test_request_i2c(void *pvParameters __attribute__((unused))) {
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


