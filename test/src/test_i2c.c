#include "test_i2c.h"

int sum = 0;

uint8_t htu21d_data[3];
uint32_t start_time = 0;
uint32_t end_time = 0;



void exti_setup(void) {
    // Habilitar la línea de interrupción EXTI0
    gpio_set_mode(TEST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, TEST_I2C_TRIGGER_PIN);
    gpio_clear(TEST_PORT, TEST_I2C_TRIGGER_PIN); // Activa la resistencia de pull-down > la raspi activa con un pulso alto

    rcc_periph_clock_enable(RCC_AFIO);

    // Configurar la interrupción para el pin INTERUPT_PIN (PB0)
    exti_select_source(EXTI0, TEST_PORT);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI0);
    
}


// Configuración del temporizador para definir la duración del test
void setup_timer(void) {
    // Configura un temporizador, por ejemplo, con una duración de 10 segundos
    rcc_periph_clock_enable(RCC_TIM2);
    // Se configura el modo de funcionamiento del Timer2
    timer_set_mode(
        TIM2,               // Timer2
        TIM_CR1_CKD_CK_INT, // fuente Clk interno
        TIM_CR1_CMS_EDGE,   // Alineado por flanco
        TIM_CR1_DIR_UP);    // Cuenta ascendente
    timer_set_prescaler(TIM2, 35999); // 72MHz / 36000 => 2KHz
    timer_set_period(TIM2, 2000 * PERIOD_INTERRUPT - 1); // 2KHz / 1000 = 2Hz => T = 0.5s 
    timer_set_counter(TIM2, 0);
    
    timer_clear_flag(TIM2, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF); // Limpiar todas las flags del temporizador
    timer_enable_irq(TIM2, TIM_DIER_UIE);   
}


void exti0_isr(void) {
    if (exti_get_flag_status(EXTI0)) {
        char aux[100];

        // Leer el valor de las flags antes de cualquier acción
        uint32_t flags_before = TIM_SR(TIM2);
        snprintf(aux, sizeof(aux), "Flags antes de set: 0x%08X\n\r", flags_before);
        print_uart(aux);
        
        // Limpiar todas las flags del temporizador
        timer_clear_flag(TIM2, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);

        // Volver a leer el valor de las flags después de limpiar
        uint32_t flags_after_clear = TIM_SR(TIM2);
        snprintf(aux, sizeof(aux), "Flags después de limpiar: 0x%08X\n\r", flags_after_clear);
        print_uart(aux);

        // Reiniciar el contador del temporizador
        timer_set_counter(TIM2, 0);
        uint32_t flags_after_set = TIM_SR(TIM2);
        snprintf(aux, sizeof(aux), "Flags después de reiniciar el contador: 0x%08X\n\r", flags_after_set);
        print_uart(aux);

        // Habilitar el temporizador
        timer_enable_counter(TIM2);
        uint32_t flags_after_enable = TIM_SR(TIM2);
        snprintf(aux, sizeof(aux), "Flags después de habilitar: 0x%08X\n\r", flags_after_enable);
        print_uart(aux);

        start_time = xTaskGetTickCount(); // Capturar el tiempo de inicio

        // Limpiar la bandera de interrupción externa
        exti_reset_request(EXTI0);
    }
}

void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        timer_clear_flag(TIM2, TIM_SR_UIF); // Limpiar la bandera del temporizador
        timer_disable_counter(TIM2);  // Detener el temporizador

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
















volatile int pulse_count = 0;  // Contador de pulsos
volatile int test_mode = 0;    // Variable para almacenar el modo de test
volatile bool timer_started = false; // Flag para controlar si el temporizador ha sido iniciado


void backup_exti0_isr(void) {
    if (exti_get_flag_status(EXTI0)) {
        // Incrementar contador de pulsos
        pulse_count++;

        // Si el temporizador no ha sido iniciado, iniciarlo con la primera interrupción
        if (!timer_started) {
            setup_timer();  // Iniciar el temporizador
            start_time = xTaskGetTickCount();
            timer_started = true;  // Marcar que el temporizador ya está iniciado
        }

        // Limpiar la bandera de interrupción
        exti_reset_request(EXTI0);
    }
}

// Interrupción del temporizador para finalizar el conteo de pulsos
void backup_tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        // Limpiar la bandera del temporizador
        timer_clear_flag(TIM2, TIM_SR_UIF);
        end_time = xTaskGetTickCount();
        // Evaluar el número de pulsos y activar diferentes tests
        /*
        if (pulse_count >= 5 && pulse_count <= 10) {
            test_mode = 1;  // Modo de test 1
        } else if (pulse_count > 10 && pulse_count <= 20) {
            test_mode = 2;  // Modo de test 2
        } else {
            test_mode = 3;  // Modo de test 3 o fallo si fuera necesario
        }
        */
        // Mostrar resultados
        // Paso a cadena de caracteres para imprimir
        //char aux[10];
        //itoa(pulse_count, aux, 10);
        //print_uart("Pulsos contados: ");
        //print_uart(aux);
        //print_uart("\n\r");
        uint32_t elapsed_time = end_time - start_time;
        float tiempo_transcurrido = (float)elapsed_time / pdMS_TO_TICKS(1000);
        

        // Mostrar el resultado
        char aux[50];
        snprintf(aux, sizeof(aux), "Tiempo pasado: %.2f ms\n\r", tiempo_transcurrido);
        print_uart(aux);

        // Reiniciar el contador de pulsos
        pulse_count = 0;
        timer_started = false;  // Permitir que el temporizador se pueda iniciar de nuevo
        timer_disable_counter(TIM2);  // Detener el temporizador

    }
}

void vLifeLineTask(void *pvParameters) {
    // No es necesario verificar GPIO en este caso, asumiendo que está configurado correctamente
    for (;;) {
        gpio_toggle(TEST_PORT, LIFE_LINE_PIN); // Alterna el estado del pin
        vTaskDelay(pdMS_TO_TICKS(LIFE_LINE_DELAY)); // Espera LIFE_LINE_DELAY segundos
    }
}

void send_test_msg_1(void) {
    char data[2] = "a";
    data[0] += sum;
    print_i2c(data, I2C_ARDUINO_ADDRESS);
    sum++;
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

/**
 * @brief Imprime un mensaje por UART
 * 
 * @param s Mensaje a imprimir
 * @return void
 */
void print_uart(const char *s){
    UART_puts(USART1, s, pdMS_TO_TICKS(500));
}