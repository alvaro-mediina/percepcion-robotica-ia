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


static const char *TAG = "MAIN";


// Publishers
rcl_publisher_t publisher_raw_angle;
rcl_publisher_t publisher_angle;


// Mensajes
std_msgs__msg__Int64 msg_raw_angle;
std_msgs__msg__Float32 msg_angle;


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


    // -------------------------
    // 1. Leer STATUS
    // -------------------------

    uint8_t status = 0;

    esp_err_t ret = as5600_read_status(&status);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo STATUS del AS5600");
        return;
    }


    // -------------------------
    // 2. Verificar iman
    // -------------------------

    if (!as5600_magnet_detected(status)) {
        ESP_LOGW(TAG, "AS5600: iman no detectado");
        return;
    }

    if (as5600_magnet_too_weak(status)) {
        ESP_LOGW(TAG, "AS5600: campo magnetico demasiado debil");
        return;
    }

    if (as5600_magnet_too_strong(status)) {
        ESP_LOGW(TAG, "AS5600: campo magnetico demasiado fuerte");
        return;
    }


    // -------------------------
    // 3. Leer RAW ANGLE
    // -------------------------

    uint16_t raw_angle = 0;

    ret = as5600_read_raw_angle(&raw_angle);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Error leyendo RAW ANGLE");
        return;
    }


    // -------------------------
    // 4. Convertir a grados
    // -------------------------

    float angle_deg =
        as5600_raw_to_degrees(raw_angle);


    // -------------------------
    // 5. Mostrar por consola
    // -------------------------

    ESP_LOGI(
        TAG,
        "STATUS=0x%02X | RAW=%u | ANGULO=%.2f grados",
        status,
        raw_angle,
        angle_deg
    );


    // -------------------------
    // 6. Publicar por micro-ROS
    // -------------------------

    msg_raw_angle.data = raw_angle;
    msg_angle.data = angle_deg;

    RCSOFTCHECK(
        rcl_publish(
            &publisher_raw_angle,
            &msg_raw_angle,
            NULL
        )
    );

    RCSOFTCHECK(
        rcl_publish(
            &publisher_angle,
            &msg_angle,
            NULL
        )
    );
}


void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator =
        rcl_get_default_allocator();

    rclc_support_t support;

    rcl_init_options_t init_options =
        rcl_get_zero_initialized_init_options();


    RCCHECK(
        rcl_init_options_init(
            &init_options,
            allocator
        )
    );


    RCCHECK(
        rmw_uros_options_set_udp_address(
            CONFIG_MICRO_ROS_AGENT_IP,
            CONFIG_MICRO_ROS_AGENT_PORT,
            rcl_init_options_get_rmw_init_options(
                &init_options
            )
        )
    );


    RCCHECK(
        rclc_support_init_with_options(
            &support,
            0,
            NULL,
            &init_options,
            &allocator
        )
    );


    // -------------------------
    // Nodo
    // -------------------------

    rcl_node_t node;

    RCCHECK(
        rclc_node_init_default(
            &node,
            "as5600_node",
            "",
            &support
        )
    );


    // -------------------------
    // Publisher RAW
    // -------------------------

    RCCHECK(
        rclc_publisher_init_default(
            &publisher_raw_angle,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Int64
            ),
            "as5600/raw_angle"
        )
    );


    // -------------------------
    // Publisher grados
    // -------------------------

    RCCHECK(
        rclc_publisher_init_default(
            &publisher_angle,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(
                std_msgs,
                msg,
                Float32
            ),
            "as5600/angle"
        )
    );


    // -------------------------
    // Timer
    // -------------------------

    rcl_timer_t timer;

    RCCHECK(
        rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(100),
            timer_callback
        )
    );


    // -------------------------
    // Executor
    // -------------------------

    rclc_executor_t executor;

    RCCHECK(
        rclc_executor_init(
            &executor,
            &support.context,
            1,
            &allocator
        )
    );

    RCCHECK(
        rclc_executor_add_timer(
            &executor,
            &timer
        )
    );


    ESP_LOGI(
        TAG,
        "micro-ROS listo. Publicando datos del AS5600..."
    );


    while (1) {

        rclc_executor_spin_some(
            &executor,
            RCL_MS_TO_NS(100)
        );

        vTaskDelay(
            pdMS_TO_TICKS(10)
        );
    }
}


void app_main(void)
{
    // -------------------------
    // Red micro-ROS
    // -------------------------

    ESP_ERROR_CHECK(
        uros_network_interface_initialize()
    );


    // -------------------------
    // AS5600 / I2C
    // -------------------------

    ESP_ERROR_CHECK(
        as5600_init()
    );


    // -------------------------
    // micro-ROS
    // -------------------------

    xTaskCreate(
        micro_ros_task,
        "uros_task",
        16000,
        NULL,
        5,
        NULL
    );
}