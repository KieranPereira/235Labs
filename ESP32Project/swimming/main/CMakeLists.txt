idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        driver               # for I2C, LEDC, etc.
        freertos             # for tasks, queues, semaphores
        nvs_flash            # for init_nvs()
        bt   
)

# enable modern C++ for just this component
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)
