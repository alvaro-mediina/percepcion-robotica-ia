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

std_msgs__msg__Int64 msg_position;
std_msgs__msg__Float32 msg_velocity;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    int64_t posicion = 0;
    float rpm = 0.0f;

    if (encoder_get_position(&posicion) == ESP_OK) {
        msg_position.data = posicion;
        RCSOFTCHECK(rcl_publish(&publisher_position, &msg_position, NULL));
    }

    if (encoder_get_speed_rpm(&rpm) == ESP_OK) {
        msg_velocity.data = rpm;
        RCSOFTCHECK(rcl_publish(&publisher_velocity, &msg_velocity, NULL));
    }
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

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(500),
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
    ESP_ERROR_CHECK(encoder_init());

    ESP_ERROR_CHECK(uros_network_interface_initialize());

    xTaskCreate(micro_ros_task,
                "uros_task",
                16000,
                NULL,
                5,
                NULL);
}

