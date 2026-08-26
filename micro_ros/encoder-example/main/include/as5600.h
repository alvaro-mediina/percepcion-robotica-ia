#ifndef AS5600_H
#define AS5600_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// Dirección I2C del AS5600
#define AS5600_I2C_ADDR 0x36

// Registros que vamos a utilizar
#define AS5600_REG_STATUS     0x0B
#define AS5600_REG_RAW_ANGLE  0x0C

// Inicializa el bus I2C
esp_err_t as5600_init(void);

// Lee la cuenta digital de 12 bits: 0 ... 4095
esp_err_t as5600_read_raw_angle(uint16_t *raw_angle);

// Convierte la cuenta digital a grados
float as5600_raw_to_degrees(uint16_t raw_angle);

// Lee el registro STATUS
esp_err_t as5600_read_status(uint8_t *status);

// Funciones auxiliares para STATUS
bool as5600_magnet_detected(uint8_t status);
bool as5600_magnet_too_weak(uint8_t status);
bool as5600_magnet_too_strong(uint8_t status);

#endif