#include "encoder.h"

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"

#define ENCODER_GPIO_A 25
#define ENCODER_GPIO_B 26

#define ENCODER_PCNT_HIGH_LIMIT 30000
#define ENCODER_PCNT_LOW_LIMIT  -30000

static const char *TAG = "ENCODER";

/*
 * Handles internos del periférico PCNT.
 * Se mantienen privados dentro de encoder.c.
 */
static pcnt_unit_handle_t encoder_unit = NULL;
static pcnt_channel_handle_t encoder_channel_a = NULL;
static pcnt_channel_handle_t encoder_channel_b = NULL;


esp_err_t encoder_init(void)
{
    esp_err_t ret;

    /*
     * 1. Crear una unidad PCNT.
     *
     * El contador PCNT es un contador con signo.
     * Los límites indican el rango permitido antes
     * de producir un evento de límite.
     */
    pcnt_unit_config_t unit_config = {
        .low_limit = ENCODER_PCNT_LOW_LIMIT,
        .high_limit = ENCODER_PCNT_HIGH_LIMIT,
    };

    ret = pcnt_new_unit(&unit_config, &encoder_unit);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo crear la unidad PCNT: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    /*
     * 2. Crear el canal asociado a la señal A.
     *
     * A se utiliza como señal de flancos.
     * B se utiliza para determinar la dirección.
     */
    pcnt_chan_config_t channel_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };

    ret = pcnt_new_channel(
        encoder_unit,
        &channel_a_config,
        &encoder_channel_a
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo crear el canal A: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    /*
     * 3. Crear el canal asociado a la señal B.
     *
     * B se utiliza como señal de flancos.
     * A se utiliza para determinar la dirección.
     */
    pcnt_chan_config_t channel_b_config = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };

    ret = pcnt_new_channel(
        encoder_unit,
        &channel_b_config,
        &encoder_channel_b
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo crear el canal B: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    /*
     * 4. Configurar las acciones del canal A.
     *
     * Flanco ascendente: incrementar.
     * Flanco descendente: decrementar.
     *
     * Cuando B está en determinado nivel,
     * se invierte la acción anterior.
     */
    ret = pcnt_channel_set_edge_action(
        encoder_channel_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando flancos del canal A: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    ret = pcnt_channel_set_level_action(
        encoder_channel_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando nivel del canal A: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    /*
     * 5. Configurar las acciones del canal B.
     *
     * Se utiliza una configuración complementaria
     * respecto del canal A.
     */
    ret = pcnt_channel_set_edge_action(
        encoder_channel_b,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando flancos del canal B: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    ret = pcnt_channel_set_level_action(
        encoder_channel_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error configurando nivel del canal B: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    /*
     * 6. Configurar un filtro contra pulsos extremadamente cortos.
     *
     * El valor debe ajustarse según:
     * - velocidad máxima del encoder;
     * - calidad de la señal;
     * - ruido eléctrico presente.
     */
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };

    ret = pcnt_unit_set_glitch_filter(
        encoder_unit,
        &filter_config
    );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo configurar el filtro PCNT: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    /*
     * 7. Habilitar, limpiar e iniciar la unidad.
     */
    ret = pcnt_unit_enable(encoder_unit);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo habilitar PCNT: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    ret = pcnt_unit_clear_count(encoder_unit);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo limpiar PCNT: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    ret = pcnt_unit_start(encoder_unit);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo iniciar PCNT: %s",
                 esp_err_to_name(ret));

        return ret;
    }

    ESP_LOGI(
        TAG,
        "Encoder inicializado: A=GPIO%d, B=GPIO%d",
        ENCODER_GPIO_A,
        ENCODER_GPIO_B
    );

    return ESP_OK;
}


esp_err_t encoder_get_count(int *count)
{
    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (encoder_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_get_count(encoder_unit, count);
}


esp_err_t encoder_clear_count(void)
{
    if (encoder_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_clear_count(encoder_unit);
}


esp_err_t encoder_stop(void)
{
    if (encoder_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_stop(encoder_unit);
}


esp_err_t encoder_start(void)
{
    if (encoder_unit == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    return pcnt_unit_start(encoder_unit);
}