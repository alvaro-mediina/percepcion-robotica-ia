idf_component_register(SRCS "encoder.c" "main.c" 
                       INCLUDE_DIRS "." "include"
                       REQUIRES driver)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)