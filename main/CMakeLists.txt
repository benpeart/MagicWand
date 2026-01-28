file(GLOB_RECURSE SRCS
    "*.c"
    "*.cc"
    "*.cpp"
)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."

    # All dependencies for all tasks in one place
    PRIV_REQUIRES
        # Core ESP-IDF subsystems
        esp_wifi
        esp_event
        esp_netif
        mqtt
        nvs_flash
        esp_timer
        driver
        spi_flash

        # Third-party libs
        esp-tflite-micro
        i2cdev
        mpu6050

        # Internal shared code
#        test_data
)

# Relax warnings for TFLM and your code
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-maybe-uninitialized
    -Wno-missing-field-initializers
    -Wno-error=sign-compare
    -Wno-error=double-promotion
    -Wno-type-limits
)
