// potenciometro.c

#include "potenciometro.h"
#include "esp_adc/adc_cali_scheme.h"

#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG_POT = "POTENCIOMETRO";

#define ADC_UNIT_USADA  ADC_UNIT_1
#define ADC_ATEN        ADC_ATTEN_DB_12   // con esta atenuación medimos hasta ~3.1-3.3V

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle;
static bool adc_calibrated = false;

// Arranca el ADC: deja listos los dos canales y la calibración de fábrica
// del chip para poder convertir las lecturas a milivolts de verdad.
void pot_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_USADA,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    // Los dos canales van configurados igual: misma atenuación y 12 bits.
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
        ESP_LOGW(TAG_POT, "No se pudo calibrar el ADC, uso conversion aproximada");
    }
}

// Lee un canal una sola vez y devuelve el resultado ya pasado a milivolts.
int adc_read_mv(adc_channel_t canal)
{
    int raw = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, canal, &raw));

    // Si tenemos calibración, la usamos para pasar de "cuentas" a mV.
    if (adc_calibrated) {
        int mv = 0;
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv));
        return mv;
    }

    // Sin calibración: regla de tres sobre los 12 bits (0..4095 -> 0..3300 mV).
    return (raw * 3300) / 4095;
}

int adc_read_mv_filtrado(adc_channel_t canal)
{
    long suma = 0;
    for (int i = 0; i < N_MUESTRAS_FILTRO; i++) {
        suma += adc_read_mv(canal);
    }
    return (int)(suma / N_MUESTRAS_FILTRO);
}

// Prepara un canal para la autocalibración.
// Arranca con un rango "supuesto" (0.3V a 3.0V) en vez de empezar de cero,
// así desde la primera lectura ya devuelve un porcentaje con sentido y no
// hace falta girar el pote de tope a tope para que "aprenda".
void pot_cal_init(pot_cal_t *cal, adc_channel_t canal)
{
    cal->canal = canal;
    cal->mv_min_visto = MV_INICIAL_MIN;
    cal->mv_max_visto = MV_INICIAL_MAX;
}

// Lee el canal (ya filtrado), ajusta el rango si hace falta y devuelve la
// posición del pote como porcentaje (0 a 100).
//

float pot_leer_pct(pot_cal_t *cal, int *mv_out)
{
    int mv = adc_read_mv_filtrado(cal->canal);
    if (mv_out) {
        *mv_out = mv;
    }

    // Acá está la autocalibración: si esta lectura es más baja o más alta
    // que lo que veníamos viendo, corremos el extremo correspondiente.
    // De a poco el rango se va acomodando al recorrido real del pote.
    if (mv < cal->mv_min_visto) {
        cal->mv_min_visto = mv;
    }
    if (mv > cal->mv_max_visto) {
        cal->mv_max_visto = mv;
    }

    // Regla de tres entre el mínimo y el máximo conocidos para sacar el %.
    int rango = cal->mv_max_visto - cal->mv_min_visto;
    if (rango <= 0) {
        return 0.0f;   // por las dudas, para no dividir por cero
    }

    float pct = (float)(mv - cal->mv_min_visto) / (float)rango * 100.0f;

    // Lo dejamos siempre entre 0 y 100, que no se escape por redondeos.
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}