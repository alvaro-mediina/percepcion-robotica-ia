#include "encoder.h"

#include "driver/pulse_cnt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

#define ENCODER_GPIO_A 25
#define ENCODER_GPIO_B 26

#define ENCODER_PCNT_HIGH_LIMIT 30000
#define ENCODER_PCNT_LOW_LIMIT  -30000

#define ENCODER_PPR 600 //pulsos por vuelta del disco
#define ENCODER_QUAD_FACTOR 4 //cuadratura x4

static const char *TAG = "ENCODER";

/*
 * Handles internos del periférico PCNT.
 * Se mantienen privados dentro de encoder.c.
 */
static pcnt_unit_handle_t encoder_unit = NULL;
static pcnt_channel_handle_t encoder_channel_a = NULL;
static pcnt_channel_handle_t encoder_channel_b = NULL;

//variables para obtener la velocidad
static int64_t encoder_wraps_acc  = 0;
static int64_t encoder_lectura_anterior = 0; //posicion de ultima vez que pedi velocidad
static int64_t encoder_tiempo_anterior = 0;  //timestamp en microsegundos de la lectura

static bool IRAM_ATTR encoder_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    encoder_wraps_acc += edata ->watch_point_value;
    return false;
}

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

    ret = pcnt_unit_add_watch_point(encoder_unit, ENCODER_PCNT_HIGH_LIMIT);

    if(ret != ESP_OK){
        ESP_LOGE(TAG, "No se pudo agregar un wath point superior: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = pcnt_unit_add_watch_point(encoder_unit, ENCODER_PCNT_LOW_LIMIT);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo agregar watch point inferior: %s", esp_err_to_name(ret));
        return ret;
    }

    pcnt_event_callbacks_t cbs = { 
        .on_reach = encoder_on_reach,
    };

    ret = pcnt_unit_register_event_callbacks(encoder_unit, &cbs, NULL);

    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "No se pudieron registrar los callbacks: %s", esp_err_to_name(ret));
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

    encoder_tiempo_anterior = esp_timer_get_time(); //Inicializa el timestamp
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

esp_err_t encoder_get_position(int64_t *position)
{
    if(position == NULL){
        return ESP_ERR_INVALID_ARG;
    }

    int raw = 0;
    esp_err_t ret = encoder_get_count(&raw);

    if(ret != ESP_OK){
        return ret;
    }

    *position =  encoder_wraps_acc + raw;

    return ESP_OK;
}

esp_err_t encoder_get_speed_rpm(float *rpm)
{
    if(rpm == NULL){
        return ESP_ERR_INVALID_ARG;
    }

    int64_t pos_actual = 0;
    esp_err_t ret = encoder_get_position(&pos_actual);

    if(ret != ESP_OK){
        return ret;
    }

    int64_t ahora_us = esp_timer_get_time();

    int64_t delta_tics = pos_actual - encoder_lectura_anterior;
    int64_t delta_us = ahora_us - encoder_tiempo_anterior;

    encoder_lectura_anterior = pos_actual;
    encoder_tiempo_anterior  = ahora_us;

    if(delta_us <=  0){
        return ESP_ERR_INVALID_STATE;
    }

    float vueltas = (float)delta_tics / (ENCODER_PPR*ENCODER_QUAD_FACTOR);
    float minutos = (delta_us/1000000.0f)/60.0f;

    *rpm = vueltas/minutos;

    return ESP_OK;
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