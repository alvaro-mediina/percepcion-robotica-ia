#include "as5600.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stddef.h>


// Pines I2C del ESP32
#define I2C_SDA_PIN GPIO_NUM_21
#define I2C_SCL_PIN GPIO_NUM_22

// Frecuencia del bus I2C
#define I2C_FREQ_HZ 400000

// Timeout para operaciones I2C
#define I2C_TIMEOUT_MS 100


static const char *TAG = "AS5600";


// Handle del bus I2C
static i2c_master_bus_handle_t i2c_bus_handle;

// Handle específico del AS5600
static i2c_master_dev_handle_t as5600_handle;


/*
 * Función auxiliar.
 * Lee uno o varios bytes empezando desde un registro.
 */
static esp_err_t as5600_read_register(
    uint8_t reg,
    uint8_t *data,
    size_t length)
{
    return i2c_master_transmit_receive(
        as5600_handle,
        &reg,
        1,
        data,
        length,
        I2C_TIMEOUT_MS
    );
}


/*
 * Inicializa:
 *
 * ESP32
 *   ↓
 * bus I2C
 *   ↓
 * dispositivo AS5600 (0x36)
 */
esp_err_t as5600_init(void)
{
    // 1. Configurar el bus I2C
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret =
        i2c_new_master_bus(
            &bus_config,
            &i2c_bus_handle
        );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo inicializar el bus I2C");
        return ret;
    }


    // 2. Configurar el AS5600 como dispositivo del bus
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AS5600_I2C_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };

    ret =
        i2c_master_bus_add_device(
            i2c_bus_handle,
            &device_config,
            &as5600_handle
        );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "No se pudo agregar el AS5600 al bus I2C");
        return ret;
    }


    // 3. Verificar si realmente responde el AS5600
    ret =
        i2c_master_probe(
            i2c_bus_handle,
            AS5600_I2C_ADDR,
            I2C_TIMEOUT_MS
        );

    if (ret != ESP_OK) {
        ESP_LOGE(
            TAG,
            "No se encontro AS5600 en direccion 0x%02X",
            AS5600_I2C_ADDR
        );

        return ret;
    }


    ESP_LOGI(
        TAG,
        "AS5600 detectado en direccion 0x%02X",
        AS5600_I2C_ADDR
    );

    return ESP_OK;
}


/*
 * Lee RAW ANGLE.
 *
 * 0x0C → bits altos
 * 0x0D → bits bajos
 *
 * Resultado:
 *
 * AAAA BBBB BBBB
 *
 * 12 bits → 0 ... 4095
 */
esp_err_t as5600_read_raw_angle(uint16_t *raw_angle)
{
    if (raw_angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];

    esp_err_t ret =
        as5600_read_register(
            AS5600_REG_RAW_ANGLE,
            data,
            2
        );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo RAW ANGLE");
        return ret;
    }


    *raw_angle =
        ((uint16_t)(data[0] & 0x0F) << 8)
        | data[1];

    return ESP_OK;
}

esp_err_t as5600_read_angle(uint16_t *angle)
{
    if (angle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2];

    esp_err_t ret =
        as5600_read_register(
            AS5600_REG_ANGLE,
            data,
            2
        );

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error leyendo ANGLE");
        return ret;
    }

    *angle =
        ((uint16_t)(data[0] & 0x0F) << 8)
        | data[1];

    return ESP_OK;
}


/*
 * Convierte:
 *
 * 0 ... 4095
 *
 * a:
 *
 * 0° ... casi 360°
 */
float as5600_raw_to_degrees(uint16_t raw_angle)
{
    return ((float) raw_angle * 360.0f) / 4096.0f;
}


/*
 * Lee STATUS (0x0B)
 *
 */

esp_err_t as5600_read_status(uint8_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return as5600_read_register(
        AS5600_REG_STATUS,
        status,
        1
    );
}


/*
 * STATUS bit 5
 *
 * MD = Magnet Detected
 */

bool as5600_magnet_detected(uint8_t status)
{
    return (status & (1U << 5)) != 0;
}


/*
 * STATUS bit 4
 *
 * ML = Magnet too weak
 */
bool as5600_magnet_too_weak(uint8_t status)
{
    return (status & (1U << 4)) != 0;
}


/*
 * STATUS bit 3
 *
 * MH = Magnet too strong
 */
bool as5600_magnet_too_strong(uint8_t status)
{
    return (status & (1U << 3)) != 0;
}