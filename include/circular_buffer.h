#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include "FreeRTOS.h"
#include <stdint.h>

typedef struct circular_buffer_t circular_buffer_t;

/**
 * @brief Crea un buffer circular de un tamaño especificado.
 * 
 * @param size Tamaño del buffer a crear. Debe ser una potencia de 2.
 * @return circular_buffer_t* Puntero al buffer creado o NULL si ocurre un error.
 */
circular_buffer_t *xCircularBufferCreate(uint16_t size);

/**
 * @brief Elimina un buffer circular liberando la memoria asignada.
 * 
 * @param cbuff Puntero al buffer a eliminar.
 */
void vCircularBufferDelete(circular_buffer_t *cbuff);

/**
 * @brief Verifica si hay datos en el buffer de recepción.
 * 
 * Se consulta cuántos datos están disponibles para leer en el buffer de recepción.
 * Debe usarse antes de intentar realizar una lectura para evitar leer datos erróneos.
 * 
 * @param cbuff Puntero al buffer circular de recepción.
 * @return uint16_t Cantidad de datos disponibles en el buffer de recepción.
 */
uint16_t serial_available(circular_buffer_t *cbuff);

/**
 * @brief Indica la cantidad de espacio disponible en el buffer de transmisión.
 * 
 * Consulta cuántos bytes están disponibles en el buffer de transmisión para enviar datos.
 * 
 * @param cbuff Puntero al buffer circular de transmisión.
 * @return uint16_t Cantidad de espacio disponible en el buffer de transmisión.
 */
uint16_t serial_sendable(circular_buffer_t *cbuff);

/**
 * @brief Escribe un dato en el buffer de transmisión de la USART.
 * 
 * Inserta un dato en el buffer de transmisión. Si no había datos pendientes,
 * inicia la transmisión.
 * 
 * @param cbuff Puntero al buffer circular de transmisión.
 * @param data Dato a transmitir.
 * @return BaseType_t pdTRUE si se pudo insertar el dato, pdFALSE si el buffer está lleno.
 */
BaseType_t serial_write(circular_buffer_t *cbuff, const uint16_t data);

/**
 * @brief Carga una cadena en el buffer de transmisión para enviar por la USART.
 * 
 * Envía una cadena de texto a través del buffer de transmisión de la USART.
 * 
 * @param cbuff Puntero al buffer circular de transmisión.
 * @param str Cadena a transmitir.
 * @return uint16_t Cantidad de caracteres cargados en el buffer.
 */
uint16_t serial_puts(circular_buffer_t *cbuff, const char *str);

/**
 * @brief Inserta un dato en el buffer circular.
 * 
 * Inserta un dato en el buffer si hay espacio disponible.
 * 
 * @param buff Puntero al buffer circular.
 * @param data Dato a insertar.
 * @return BaseType_t pdTRUE si se pudo insertar el dato, pdFALSE si el buffer está lleno.
 */
BaseType_t push(circular_buffer_t *buff, const uint16_t data);

/**
 * @brief Lee un dato del buffer de recepción.
 * 
 * Extrae el dato más antiguo disponible en el buffer de recepción.
 * 
 * @param buff Puntero al buffer circular.
 * @param data Puntero donde se almacenará el dato extraído.
 * @return BaseType_t pdPASS si se pudo extraer el dato, pdFAIL si el buffer está vacío.
 */
BaseType_t pop(circular_buffer_t *buff, uint16_t *data);

/* Otras declaraciones de funciones aquí */

/**
 * @brief Devuelve en una cadena los datos no leídos del buffer.
 * 
 * Lee los datos no leídos del buffer circular sin modificar los índices.
 * 
 * @param buff Puntero al buffer circular.
 * @param output Puntero a la cadena donde se almacenarán los datos no leídos.
 * @param max_len Tamaño máximo de la cadena para evitar desbordamientos.
 * @return uint16_t Cantidad de datos copiados a la cadena.
 */
uint16_t circular_buffer_to_string(circular_buffer_t *buff, char *output, uint16_t max_len);

/**
 * @brief Resetea el buffer circular.
 * 
 * Vacía el buffer circular al reiniciar los índices `in` y `out`.
 * 
 * @param buff Puntero al buffer circular a resetear.
 */
void circular_buffer_reset(circular_buffer_t *buff);


/**
 * @brief Copia una subcadena de la cadena fuente a una cadena de destino, delimitada por patrones de inicio y fin.
 *
 * La función `Copy_from_to` busca un patrón de inicio en la cadena fuente, y luego copia todo el contenido
 * entre el patrón de inicio y el patrón de fin en el buffer de destino, incluyendo tanto el patrón de inicio 
 * como el patrón de fin.
 * Si alguno de los patrones no se encuentra en la cadena fuente o si el buffer de destino no es lo suficientemente 
 * grande para contener la subcadena, la función retorna un código de error.
 *
 * @param source         Puntero a la cadena fuente donde se buscarán los patrones y la subcadena a copiar.
 * @param pattern_start  Puntero al patrón de inicio que indica desde dónde se copiará la subcadena.
 * @param pattern_finish Puntero al patrón de fin que indica hasta dónde se copiará la subcadena.
 * @param dest           Puntero al buffer de destino donde se copiará la subcadena extraída.
 * @param dest_size      Tamaño máximo del buffer de destino.
 *
 * @return int
 *         0  - Éxito, la subcadena fue copiada correctamente al buffer de destino.
 *        -1  - El patrón de inicio no se encontró en la cadena fuente.
 *        -2  - El patrón de fin no se encontró en la cadena fuente después del patrón de inicio.
 *        -3  - El buffer de destino es demasiado pequeño para contener la subcadena.
 *
 * @note Esta función copia el patrón de inicio y el patrón de fin en el buffer de destino.
 * @note La función asume que el buffer `dest` es lo suficientemente grande para almacenar la subcadena resultante,
 *       incluyendo el carácter nulo ('\0') al final de la cadena. Si el buffer no es suficiente, se retorna -3.
 */
int Copy_from_to(const char *source, const char *pattern_start, const char *pattern_finish, char *dest, size_t dest_size);

#endif // CIRCULAR_BUFFER_H