#include "motor.h"
#include "co2sensor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

void motor_task(void *pvParameters) {
    while (1) {
        for (int angle = 0; angle <= 180; angle += 10) {
            motor_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        for (int angle = 180; angle >= 0; angle -= 10) {
            motor_set_angle(angle);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void co2sensor_task(void *pvParameters){    // Initialize the CO2 sensor
    co2sensor_init();

    // Calibrate the CO2 sensor
    co2sensor_calibrate(400);  // Example calibration with 400 ppm

    // Enable ABOC
    co2sensor_enable_aboc();

    // Main loop
    while (1) {
        if (co2sensor_is_data_ready()) {
            uint16_t co2_ppm = co2sensor_read_co2();
            printf("CO2 Concentration: %d ppm\n", co2_ppm);
        } else {
            printf("CO2 data not ready...\n");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 1 second
    }
}

void app_main(void) {
    motor_init();
    
    // Create FreeRTOS tasks for motor and CO2 sensor
    xTaskCreate(motor_task, "Motor Task", 2048, NULL, 5, NULL);
    xTaskCreate(co2sensor_task, "CO2 Sensor Task", 2048, NULL, 5, NULL);
}