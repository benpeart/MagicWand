
#
# Main component of TF Micro project 'micro_speech'.
#

file(GLOB_RECURSE SRCS "*.cc" "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    PRIV_REQUIRES spi_flash driver esp_timer test_data i2cdev mpu6050
    INCLUDE_DIRS "")

    # Reduce the level of paranoia to be able to compile sources
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-maybe-uninitialized
    -Wno-missing-field-initializers
    -Wno-error=sign-compare
    -Wno-error=double-promotion
    -Wno-type-limits)
