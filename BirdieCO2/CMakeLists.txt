idf_component_register(
    SRCS "src/main.c" "src/motor.c" "src/co2sensor.c"
    INCLUDE_DIRS "inc"
    REQUIRES driver
)