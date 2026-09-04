#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uros_network_interfaces.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/float32.h>

#include "as5600.h"
#include "potenciometro.h"


#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if (temp_rc != RCL_RET_OK) { \
        ESP_LOGE(TAG, "Fallo en %s, linea %d, error %d", \
                 __FILE__, __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if (temp_rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "Fallo leve en linea %d, error %d", \
                 __LINE__, (int)temp_rc); \
    } \
}


static const char *TAG = "SENSORES_NODE";

// AS5600 disponible en runtime? (permite arrancar sin el sensor conectado)
static bool as5600_disponible = false;


// -------------------------
// Publishers (6 en total)
// -------------------------
rcl_publisher_t publisher_raw_angle;
rcl_publisher_t publisher_angle;

rcl_publisher_t publisher_pot_voltaje_acond;
rcl_publisher_t publisher_pot_posicion_acond;
rcl_publisher_t publisher_pot_voltaje_crudo;
rcl_publisher_t publisher_pot_posicion_crudo;


// -------------------------
// Mensajes
// -------------------------
std_msgs__msg__Int64 msg_raw_angle;
std_msgs__msg__Float32 msg_angle;

std_msgs__msg__Float32 msg_pot_voltaje_acond;
std_msgs__msg__Float32 msg_pot_posicion_acond;
std_msgs__msg__Float32 msg_pot_voltaje_crudo;
std_msgs__msg__Float32 msg_pot_posicion_crudo;


// Estados de calibración del potenciómetro
static pot_cal_t cal_cruda;
static pot_cal_t cal_acond;



void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    // ============ AS5600 (solo si está disponible) ============
    if (as5600_disponible) {
        uint8_t status = 0;
        esp_err_t ret = as5600_read_status(&status);

        if (ret == ESP_OK && as5600_magnet_detected(status)
            && !as5600_magnet_too_weak(status)
            && !as5600_magnet_too_strong(status)) {

            uint16_t raw_angle = 0;
            ret = as5600_read_raw_angle(&raw_angle);

            if (ret == ESP_OK) {
                float angle_deg = as5600_raw_to_degrees(raw_angle);

                msg_raw_angle.data = raw_angle;
                msg_angle.data = angle_deg;

                RCSOFTCHECK(rcl_publish(&publisher_raw_angle, &msg_raw_angle, NULL));
                RCSOFTCHECK(rcl_publish(&publisher_angle, &msg_angle, NULL));
            } else {
                ESP_LOGW(TAG, "Error leyendo RAW ANGLE");
            }
        } else {
            ESP_LOGW(TAG, "AS5600: iman no detectado o fuera de rango");
        }
    }

    // ============ Potenciómetro (siempre corre) ============
    int mv_cruda = 0, mv_acond = 0;
    float pct_cruda = pot_leer_pct(&cal_cruda, &mv_cruda);
    float pct_acond = pot_leer_pct(&cal_acond, &mv_acond);

    msg_pot_voltaje_crudo.data  = mv_cruda / 1000.0f;
    msg_pot_posicion_crudo.data = pct_cruda;
    msg_pot_voltaje_acond.data  = mv_acond / 1000.0f;
    msg_pot_posicion_acond.data = pct_acond;

    RCSOFTCHECK(rcl_publish(&publisher_pot_voltaje_crudo,  &msg_pot_voltaje_crudo,  NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_posicion_crudo, &msg_pot_posicion_crudo, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_voltaje_acond,  &msg_pot_voltaje_acond,  NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_posicion_acond, &msg_pot_posicion_acond, NULL));

    // ============ Log combinado por consola ============
    if (as5600_disponible) {
        printf("AS5600: RAW=%lld ANGLE=%.2f | Pot: %d mV (%.1f%%) / %d mV (%.1f%%)\n",
               msg_raw_angle.data, msg_angle.data,
               mv_cruda, pct_cruda, mv_acond, pct_acond);
    } else {
        printf("Pot: %d mV (%.1f%%) / %d mV (%.1f%%)\n",
               mv_cruda, pct_cruda, mv_acond, pct_acond);
    }
}


void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    // Domain ID único y explícito, para no depender de config externa
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 10));

    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rcl_init_options_get_rmw_init_options(&init_options)));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "sensores_node", "", &support));

    // ---- Publishers AS5600 ----
    RCCHECK(rclc_publisher_init_default(
        &publisher_raw_angle, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "as5600/raw_angle"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_angle, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "as5600/angle"));

    // ---- Publishers Potenciómetro ----
    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_voltaje_crudo, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/voltaje_crudo"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_posicion_crudo, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/posicion_cruda"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_voltaje_acond, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/voltaje_acondicionado"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_posicion_acond, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/posicion_acondicionada"));

    // ---- Timer único, 100ms ----
    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(100), timer_callback));

    // ---- Executor ----
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(TAG, "micro-ROS listo. Publicando sensores...");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    // ---- ADC del potenciómetro (antes del xTaskCreate, por el orden que ya sabés) ----
    pot_adc_init();
    pot_cal_init(&cal_cruda, CH_CRUDA);
    pot_cal_init(&cal_acond, CH_ACONDICIONADA);

    // ---- AS5600 (opcional: si falla, seguimos solo con el potenciómetro) ----
    esp_err_t as5600_ret = as5600_init();
    if (as5600_ret == ESP_OK) {
        as5600_disponible = true;
        ESP_LOGI(TAG, "AS5600 detectado y disponible");
    } else {
        as5600_disponible = false;
        ESP_LOGW(TAG, "AS5600 no disponible, se publicara solo el potenciometro");
    }

    // ---- micro-ROS ----
    xTaskCreate(micro_ros_task, "uros_task", 16000, NULL, 5, NULL);
}