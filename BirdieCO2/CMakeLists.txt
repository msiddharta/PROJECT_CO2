idf_component_register(
    SRCS "src/main.c" "src/motor.c" "src/co2sensor.c" "src/battery.c" "src/max17048_adv.c" "src/i2c_bus.c" "src/mqtt_client.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver
)