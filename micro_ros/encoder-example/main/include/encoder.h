#ifndef ENCODER_H
#define ENCODER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Inicializa el periférico PCNT para leer el encoder.
 */
esp_err_t encoder_init(void);

/**
 * @brief Obtiene la cuenta acumulada del encoder.
 *
 * @param count Dirección donde se guardará el resultado.
 */
esp_err_t encoder_get_count(int *count);

/**
 * @brief Reinicia la cuenta del encoder a cero.
 */
esp_err_t encoder_clear_count(void);

/**
 * @brief Detiene el contador PCNT.
 */
esp_err_t encoder_stop(void);

/**
 * @brief Inicia o reanuda el contador PCNT.
 */
esp_err_t encoder_start(void);

esp_err_t encoder_get_position(int64_t *position);

esp_err_t encoder_get_speed_rpm(float *rpm);

#ifdef __cplusplus
}
#endif

#endif