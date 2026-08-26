// potenciometro.c

#include "potenciometro.h"
#include "esp_adc/adc_cali_scheme.h"

#include <stdbool.h>       // NUEVO
#include "esp_err.h"       // NUEVO
#include "esp_log.h"       // NUEVO, para el log de calibración

static const char *TAG_POT = "POTENCIOMETRO";

#define ADC_UNIT_USADA  ADC_UNIT_1 
#define ADC_ATEN        ADC_ATTEN_DB_12

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;
static bool adc_calibrated = false;

void pot_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_USADA,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, CH_ACONDICIONADA, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, CH_CRUDA, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_USADA,
        .atten = ADC_ATEN,
        .bitwidth = ADC_BITWIDTH_12,
        .default_vref = 1100,
    };
    esp_err_t ret = adc_cali_create_scheme_line_fitting(&cali_cfg, &adc_cali_handle);
    adc_calibrated = (ret == ESP_OK);

    if (!adc_calibrated) {
        ESP_LOGW(TAG_POT, "Calibracion ADC no disponible, usando conversion cruda");
    }
}

int adc_read_mv_filtrado(adc_channel_t canal)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, canal, &raw));

    if (adc_calibrated) {
        int mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv));
        return mv;
    } else {
        return (raw * 3300) / 4095;
    }
}

#define V_MAX_MV  3300.0f

float mv_a_posicion_pct(int mv)
{
    float pct = (mv / V_MAX_MV) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}