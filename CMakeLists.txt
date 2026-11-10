idf_component_register(
    SRCS "src/ST25R3916.cpp"
    INCLUDE_DIRS "src"
    REQUIRES esp_driver_i2c esp_timer freertos
)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
