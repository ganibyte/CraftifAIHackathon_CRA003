idf_component_register(
    SRCS main.c imu_mpu6050.c ble_beacon.c board.c
    INCLUDE_DIRS "."
    REQUIRES wt_core bt nvs_flash esp_driver_i2c esp_driver_gpio esp_timer esp_pm esp_hw_support
             esp_system freertos log
)
