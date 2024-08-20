#include "FreeRTOS.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "test.h"

#include "uart1.h"
#include "uart2.h"

#include "libopencm3/stm32/rcc.h"

void taskTest(void *args __attribute__((unused))){
    char string_1[] = "Testing UART 1\r\n";
    int len_string_1 = strlen(string_1);

    uint16_t nsent_1 = UART1_puts(string_1);

    if(nsent_1 != sizeof(string_1) - 1){
        UART1_puts("Error al enviar datos por UART1\r\n");
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    int data;
    int i = 0;
    while((data = UART1_receive()) != -1){
        if(string_1[i] != (char)data || i == len_string_1) {
            UART2_puts("Error al recibir datos por UART1.\r\n");
            vTaskDelete(NULL);
        }
        i++;
    }

    UART2_puts("Test UART1 runned successfully\r\n");
    vTaskDelete(NULL);
}


