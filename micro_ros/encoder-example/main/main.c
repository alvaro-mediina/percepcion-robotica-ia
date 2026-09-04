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

#include <std_msgs/msg/float32.h>

#include "potenciometro.h"


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

// Los cuatro publishers: voltaje y posición, para la señal cruda y la acondicionada.
rcl_publisher_t publisher_pot_voltaje_acond;
rcl_publisher_t publisher_pot_posicion_acond;
rcl_publisher_t publisher_pot_voltaje_crudo;
rcl_publisher_t publisher_pot_posicion_crudo;

// Los mensajes que vamos a ir llenando y publicando. Todos son Float32.
std_msgs__msg__Float32 msg_pot_voltaje_acond;
std_msgs__msg__Float32 msg_pot_posicion_acond;
std_msgs__msg__Float32 msg_pot_voltaje_crudo;
std_msgs__msg__Float32 msg_pot_posicion_crudo;

// Un "estado de calibración" por cada canal. Cada uno se va acomodando solo
// al rango real de su señal a medida que movemos el pote.
static pot_cal_t cal_cruda;
static pot_cal_t cal_acond;

// Esta función se ejecuta sola cada vez que salta el timer (cada 100 ms).
// Lee los dos canales, calcula posición y voltaje, lo muestra y lo publica.
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    // Leemos cada canal. pot_leer_pct nos devuelve el porcentaje y de paso
    // nos deja el voltaje en mv_cruda / mv_acond.
    int mv_cruda = 0, mv_acond = 0;
    float pct_cruda = pot_leer_pct(&cal_cruda, &mv_cruda);
    float pct_acond = pot_leer_pct(&cal_acond, &mv_acond);

    // Un print para ir viendo en el monitor qué está midiendo.
    printf("Cruda: %d mV (%.1f%%)  |  Acondicionada: %d mV (%.1f%%)\n",
        mv_cruda, pct_cruda, mv_acond, pct_acond);

    // Cargamos los mensajes. El voltaje va en volts, así que pasamos de mV a V.
    msg_pot_voltaje_crudo.data  = mv_cruda / 1000.0f;
    msg_pot_posicion_crudo.data = pct_cruda;
    msg_pot_voltaje_acond.data  = mv_acond / 1000.0f;
    msg_pot_posicion_acond.data = pct_acond;

    // Y los publicamos en sus tópicos.
    RCSOFTCHECK(rcl_publish(&publisher_pot_voltaje_crudo,  &msg_pot_voltaje_crudo,  NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_posicion_crudo, &msg_pot_posicion_crudo, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_voltaje_acond,  &msg_pot_voltaje_acond,  NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pot_posicion_acond, &msg_pot_posicion_acond, NULL));
}

// Esta es la tarea principal de micro-ROS: arma la conexión con el agente,
// crea el nodo, los publishers y el timer, y después se queda girando.
void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    // Conexión con el micro-ROS Agent por UDP (la IP y el puerto salen del menuconfig).
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP,
        CONFIG_MICRO_ROS_AGENT_PORT,
        rcl_init_options_get_rmw_init_options(&init_options)));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "potenciometro_node", "", &support));

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

    // Preparamos el ADC antes de arrancar a leer.
    pot_adc_init();

    // Dejamos cada canal listo para autocalibrarse. Arrancan con el rango
    // supuesto (0.3V a 3.0V) y se van ajustando solos con el uso.
    pot_cal_init(&cal_cruda, CH_CRUDA);
    pot_cal_init(&cal_acond, CH_ACONDICIONADA);

    // Y lanzamos la tarea de micro-ROS.
    xTaskCreate(micro_ros_task,
                "uros_task",
                16000,
                NULL,
                5,
                NULL);
}