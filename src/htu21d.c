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
        return pdFALSE;
    }

    for (int retry_count = 0; retry_count < 3; retry_count++) {
        uint8_t command = SOFT_RESET;
        if(i2c_send_to_slave(i2c_id, HTU21D_ADDRESS, &command, 1) == I2C_PASS){
            vTaskDelay(pdMS_TO_TICKS(15));
            return pdTRUE;  // Comunicación exitosa
        } else{
            vTaskDelay(pdMS_TO_TICKS(100));  // Espera antes del siguiente intento
        }
    }
   
    return pdFALSE;

}

/**
 * @brief Procedimiento para solicitar la temperatura al sensor HTU21D
 * 
 * @param i2c_id Identificador de 32 bits del periférico I2C (I2C1 o I2C2)
 * @param command Comando para solicitar la temperatura
 * @param data Puntero a un arreglo de 3 bytes para almacenar la respuesta
 * 
 * @return pdPASS si la solicitud fue exitosa, pdFALSE si hubo un error
 */

BaseType_t request_htu21d(uint32_t i2c_id, uint8_t command, uint8_t *data){
    if(data == NULL){
        return pdFALSE;
    }

    i2c_t *i2c = get_i2c(i2c_id);
    if (i2c == NULL) {
        return pdFALSE;
    }

    if(i2c_send_to_slave(i2c_id, HTU21D_ADDRESS, &command, 1) != I2C_PASS){
        return pdFALSE;
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Espera suficiente para la lectura

    if(i2c_read_from_slave(i2c_id, HTU21D_ADDRESS, 3, data) != I2C_PASS){
        return pdFALSE;
    }

    return pdPASS;
}