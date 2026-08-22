idf_component_register(SRCS "encoder.c" "main.c" "potenciometro.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES driver esp_adc micro_ros_espidf_component
                       PRIV_REQUIRES esp_timer)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)