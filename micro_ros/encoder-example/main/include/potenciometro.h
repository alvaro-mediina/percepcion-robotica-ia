// potenciometro.h

#pragma once

#include "hal/adc_types.h"        // NUEVO: para adc_channel_t
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

#define CH_ACONDICIONADA  ADC_CHANNEL_7   // GPIO35 - señal después del op-amp
#define CH_CRUDA          ADC_CHANNEL_6   // GPIO34 - señal directa del pot

#define POT_FILTRO_N      10   // cantidad de muestras del promedio movil

void pot_adc_init(void);
int adc_read_mv(adc_channel_t canal);
int adc_read_mv_filtrado(adc_channel_t canal);
float mv_a_posicion_pct(int mv);