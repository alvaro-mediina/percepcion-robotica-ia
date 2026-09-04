// potenciometro.h
#ifndef POTENCIOMETRO_H
#define POTENCIOMETRO_H

#include "esp_adc/adc_oneshot.h"



#define CH_CRUDA          ADC_CHANNEL_6   // GPIO34, la señal directa del pote
#define CH_ACONDICIONADA  ADC_CHANNEL_7   // GPIO35, la señal que pasó por el LM324


#define N_MUESTRAS_FILTRO   32


#define MV_INICIAL_MIN   500    // 0.3 V
#define MV_INICIAL_MAX   1.5   // 3.0 V

// Guarda todo lo que la autocalibración necesita saber de un canal:
// qué canal es y cuáles son el mínimo y el máximo que fue viendo.
typedef struct {
    adc_channel_t canal;
    int mv_min_visto;
    int mv_max_visto;
} pot_cal_t;

// Funciones que expone este módulo
void  pot_adc_init(void);
int   adc_read_mv(adc_channel_t canal);            // una lectura suelta, en mV
int   adc_read_mv_filtrado(adc_channel_t canal);   // promedio de varias, en mV

void  pot_cal_init(pot_cal_t *cal, adc_channel_t canal);  // prepara un canal
float pot_leer_pct(pot_cal_t *cal, int *mv_out);          // lee y devuelve el %

    #endif // POTENCIOMETRO_H