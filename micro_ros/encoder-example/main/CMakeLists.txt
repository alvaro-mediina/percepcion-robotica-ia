idf_component_register(
    SRCS
        "encoder.c"
        "as5600.c"
        "potenciometro.c"
        "main.c"

    INCLUDE_DIRS
        "."
        "include"

    REQUIRES
        driver
        esp_adc
        esp_driver_i2c
        micro_ros_espidf_component

    PRIV_REQUIRES
        esp_timer
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)