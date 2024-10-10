#include "htu21d.h"

/**
 * @brief Resetea el sensor HTU21D
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @return pdTRUE si la comunicación fue exitosa, pdFALSE si hubo un error
 */

BaseType_t reset_htu21d(uint32_t i2c_id){
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    for (int retry_count = 0; retry_count < 3; retry_count++) {
        if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (i2c_start(i2c->i2c_id, HTU21D_ADDRESS, false) == pdPASS) {
                i2c_write(i2c->i2c_id, SOFT_RESET);
                i2c_send_stop(i2c->i2c_id);
                vTaskDelay(pdMS_TO_TICKS(15));
                xSemaphoreGive(i2c->mutex);
                vTaskDelay(pdMS_TO_TICKS(15));
                return pdTRUE;  // Comunicación exitosa
            } else {
                xSemaphoreGive(i2c->mutex);
                vTaskDelay(pdMS_TO_TICKS(100));  // Espera antes del siguiente intento
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));  // Espera antes del siguiente intento
        }
    }
   
    return pdFALSE;

}

/**
 * @brief Procedimiento para solicitar la temperatura al sensor HTU21D
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * 
 * @return pdPASS si la solicitud fue exitosa, pdFALSE si hubo un error
 */

BaseType_t request_htu21d(uint32_t i2c_id, uint8_t command) {
    BaseType_t status;
    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        print_uart("Error: No se pudo obtener el periférico I2C.\n\r");
        return pdFALSE;
    }

    if (xSemaphoreTake(i2c->mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (i2c_start(i2c->i2c_id, HTU21D_ADDRESS, false) == pdPASS) {
            i2c_write(i2c->i2c_id, command);
            i2c_send_stop(i2c->i2c_id);
        } else {
            print_uart("Error al iniciar comunicacion (comando).\n\r");
            xSemaphoreGive(i2c->mutex);
            return pdFALSE;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Espera suficiente para la lectura

        if(i2c_make_request(i2c_id, HTU21D_ADDRESS, 3) != pdPASS){
            print_uart("Error: No se pudo realizar la solicitud.\n\r");
            xSemaphoreGive(i2c->mutex);
            return pdFALSE;
        }

        xSemaphoreGive(i2c->mutex);
        
    } else {
        print_uart("Error: No se pudo obtener el mutex.\n\r");
        return pdFALSE;
    }

    return pdPASS;
}