idf_component_register(
    SRCS "src/main.c" "src/motor.c" "src/co2sensor.c" "src/battery.c" "src/max17048_adv.c" "src/i2c_bus.c" "src/mqtt.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver esp_wifi esp_event nvs_flash mqtt
)