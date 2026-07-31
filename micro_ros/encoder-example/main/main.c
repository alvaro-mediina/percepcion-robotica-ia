#include "encoder.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAIN";

void app_main(void)
{

    ESP_ERROR_CHECK(encoder_init());

    while (1) {
        int pulse_count = 0;

        esp_err_t result = encoder_get_count(&pulse_count);

        if (result == ESP_OK) {
            ESP_LOGI(TAG, "Cuenta del encoder: %d", pulse_count);
        } else {
            ESP_LOGE(
                TAG,
                "Error leyendo el encoder: %s",
                esp_err_to_name(result)
            );
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}