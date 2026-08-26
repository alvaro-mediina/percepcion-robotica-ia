#include "encoder.h"

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

#include "potenciometro.h"   // ajustá el nombre si tu header se llama distinto

#define RCCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGE(TAG, "Fallo en %s, linea %d, error %d", __FILE__, __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); \
    } \
}

#define RCSOFTCHECK(fn) { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        ESP_LOGW(TAG, "Fallo leve en linea %d, error %d", __LINE__, (int)temp_rc); \
    } \
}

static const char *TAG = "MAIN";

rcl_publisher_t publisher_position;
rcl_publisher_t publisher_velocity;

// NUEVO: publishers del potenciómetro
rcl_publisher_t publisher_pot_voltaje_acond;
rcl_publisher_t publisher_pot_posicion_acond;
rcl_publisher_t publisher_pot_voltaje_crudo;
rcl_publisher_t publisher_pot_posicion_crudo;

std_msgs__msg__Int64 msg_position;
std_msgs__msg__Float32 msg_velocity;

// NUEVO: mensajes del potenciómetro
std_msgs__msg__Float32 msg_pot_voltaje_acond;
std_msgs__msg__Float32 msg_pot_posicion_acond;
std_msgs__msg__Float32 msg_pot_voltaje_crudo;
std_msgs__msg__Float32 msg_pot_posicion_crudo;

void timer_callback(
    rcl_timer_t *timer,
    int64_t last_call_time,
    unsigned int missed_calls)
{
    RCLC_UNUSED(last_call_time);
    RCLC_UNUSED(missed_calls);

    if (timer == NULL) {
        return;
    }

    int mv_acond = adc_read_mv_filtrado(CH_ACONDICIONADA);
    int mv_cruda = adc_read_mv_filtrado(CH_CRUDA);

    float pct_acond = mv_a_posicion_pct(mv_acond);
    float pct_cruda = mv_a_posicion_pct(mv_cruda);

    printf(
        "Cruda: %d mV (%.1f%%) | Acondicionada: %d mV (%.1f%%)\n",
        mv_cruda,
        pct_cruda,
        mv_acond,
        pct_acond
    );

    msg_pot_voltaje_acond.data = mv_acond / 1000.0f;
    msg_pot_posicion_acond.data = pct_acond;

    msg_pot_voltaje_crudo.data = mv_cruda / 1000.0f;
    msg_pot_posicion_crudo.data = pct_cruda;

    RCSOFTCHECK(rcl_publish(&publisher_pot_voltaje_acond, &msg_pot_voltaje_acond, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_posicion_acond, &msg_pot_posicion_acond, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_voltaje_crudo, &msg_pot_voltaje_crudo, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_posicion_crudo, &msg_pot_posicion_crudo, NULL));
}

void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rcl_init_options_get_rmw_init_options(&init_options)));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "encoder_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &publisher_position,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
        "encoder/position"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_velocity,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "encoder/velocity"));

    // NUEVO: publishers del potenciómetro
    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_voltaje_acond,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/voltaje_acondicionado"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_posicion_acond,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/posicion_acondicionada"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_voltaje_crudo,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/voltaje_crudo"));

    RCCHECK(rclc_publisher_init_default(
        &publisher_pot_posicion_crudo,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "potenciometro/posicion_cruda"));

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    ESP_LOGI(TAG, "micro-ROS listo, publicando...");

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    pot_adc_init();   // primero: inicializar el ADC (arreglado el orden)

    xTaskCreate(micro_ros_task,
                "uros_task",
                16000,
                NULL,
                5,
                NULL);
}