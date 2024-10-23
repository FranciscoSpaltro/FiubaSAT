#include "test_i2c.h"

uint8_t htu21d_data[3];
uint32_t start_time = 0;
uint32_t end_time = 0;
volatile int pulse_count = 0;  // Contador de pulsos
volatile int test_mode = 0;    // Variable para almacenar el modo de test
volatile bool timer_started = false; // Flag para controlar si el temporizador ha sido iniciado

void echo_setup(void) {
    rcc_periph_clock_enable(ECHO_LINE_RCC);

    gpio_set_mode(
        ECHO_LINE_PORT, 
        GPIO_MODE_OUTPUT_2_MHZ,
        GPIO_CNF_OUTPUT_PUSHPULL,
        ECHO_LINE_PIN
    );

    gpio_set(ECHO_LINE_PORT, ECHO_LINE_PIN); // Establece el pin alto
}

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
    timer_enable_counter(TIM2); // Va a saltar al encender pero ya queda andando bien para todas las ejecuciones (corregir)
}


void exti0_isr(void) {
    if (exti_get_flag_status(EXTI0)) {
        // Incrementar contador de pulsos
        pulse_count++;

        // Si el temporizador no ha sido iniciado, iniciarlo con la primera interrupción
        if (!timer_started) {
            timer_set_counter(TIM2, 0);
            timer_clear_flag(TIM2, TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF | TIM_SR_CC3IF | TIM_SR_CC4IF);
            timer_enable_counter(TIM2);
            start_time = xTaskGetTickCount();
            timer_started = true;  // Marcar que el temporizador ya está iniciado
        }

        // Limpiar la bandera de interrupción
        exti_reset_request(EXTI0);
    }
}

void send_echo_pulses(int n){
    if(n == 0 || n > PERIOD_INTERRUPT * 1000) return;
    gpio_set(ECHO_LINE_PORT, ECHO_LINE_PIN); // Establece el pin alto
    for (int j = 0; j < n; j++) {
        gpio_clear(ECHO_LINE_PORT, ECHO_LINE_PIN); // Establece el pin alto
        vTaskDelay(pdMS_TO_TICKS(1)); // Espera LIFE_LINE_DELAY segundos
        gpio_set(ECHO_LINE_PORT, ECHO_LINE_PIN); // Establece el pin alto
        vTaskDelay(pdMS_TO_TICKS(1)); // Espera LIFE_LINE_DELAY segundos
    }
}
// Interrupción del temporizador para finalizar el conteo de pulsos
void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_UIF)) {
        // Limpiar la bandera del temporizador
        timer_disable_counter(TIM2);
        timer_clear_flag(TIM2, TIM_SR_UIF);
        end_time = xTaskGetTickCount();

        // Mostrar resultados
        char aux[50];
        itoa(pulse_count, aux, 10);
        print_uart("Pulsos contados: ");
        print_uart(aux);
        print_uart("\n\r");

        uint32_t elapsed_time = end_time - start_time;
        float tiempo_transcurrido = (float)elapsed_time / pdMS_TO_TICKS(1000);
        

        // Mostrar el resultado
        snprintf(aux, sizeof(aux), "Tiempo: %.2f ms\n\r", tiempo_transcurrido);
        print_uart(aux);

        switch (pulse_count){
            case 1:
                send_echo_pulses(pulse_count);
                break;
            case 2:
                send_test_msg_1();
                break;

            case 3:
                send_test_msg_2();
                break;
            
            default:
                print_uart("Pulso no reconocido\n\r");
                break;
        }

        pulse_count = 0;
        timer_started = false;  // Permitir que el temporizador se pueda iniciar de nuevo

    }
}

void send_test_msg_1(void) {
    char data[2] = "a";
    print_i2c(data, I2C_ARDUINO_ADDRESS);
}

void send_test_msg_2(void) {
    char data[50] = "Prueba extensa numero 1";
    print_i2c(data, I2C_ARDUINO_ADDRESS);
    sprintf(data, "Prueba extensa numero 2");
    print_i2c(data, I2C_ARDUINO_ADDRESS);
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