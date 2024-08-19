#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include "test.h"

#include "uart1.h"
#include "uart2.h"

#include "libopencm3/stm32/rcc.h"

void taskTest(void *args __attribute__((unused))){
    char string_1[] = "Testing UART 1\r\n";
    char string_2[] = "Testing UART 2\r\n";

    uint16_t nsent_1 = UART1_puts(string_1);
    uint16_t nsent_2 = UART2_puts(string_2);

    if(nsent_1 != sizeof(string_1) - 1){
        UART1_puts("Error al enviar datos por UART1\r\n");
        vTaskDelete(NULL);
    }

    if(nsent_2 != sizeof(string_2) - 1){
        UART1_puts("Error al enviar datos por UART2\r\n");
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
    UART2_get_buffer();
    UART1_puts(UART2_get_buffer());
    UART1_puts("Tests runned successfully\r\n");
    vTaskDelete(NULL);
}


