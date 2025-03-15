    #include "motor.h"
    #include "co2sensor.h"
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "freertos/event_groups.h"
    #include <stdio.h>

    // Define event group
    #define CO2_HIGH_EVENT (1 << 0)  // Event bit for high CO2 level
    #define CO2_LOW_EVENT  (1 << 1)  // Event bit for low CO2 level

    static EventGroupHandle_t xCO2EventGroup;

    void motor_task(void *pvParameters) {
        while (1) {
            // Wait for an event indicating CO2 level change
            EventBits_t uxBits = xEventGroupWaitBits(xCO2EventGroup, CO2_HIGH_EVENT | CO2_LOW_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

            // Check which event occurred
            if (uxBits & CO2_HIGH_EVENT) {
                printf("CO2 level is high, setting motor to 90°.\n");
                motor_set_angle(90);  // Set motor to 90° if CO2 > 1000 ppm
            } 
            if (uxBits & CO2_LOW_EVENT) {
                printf("CO2 level is safe, setting motor to 0°.\n");
                motor_set_angle(0);   // Set motor to 0° if CO2 < 1000 ppm
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay to avoid tight loop
        }
    }

    void co2sensor_task(void *pvParameters) {
        // Initialize the CO2 sensor
        co2sensor_init();

        // Calibrate the CO2 sensor
        co2sensor_calibrate(400);  // Example calibration with 400 ppm

        // Enable ABOC (Automatic Baseline Offset Correction)
        co2sensor_enable_aboc();

        // Variable to store CO2 concentration
        uint16_t co2_ppm;

        // Main loop for reading the CO2 sensor
    /* while (1) {
            if (co2sensor_is_data_ready()) {
                co2_ppm = co2sensor_read_co2();
                printf("CO2 Concentration: %d ppm\n", co2_ppm);

                // Check CO2 concentration and set the corresponding flag
                if (co2_ppm > 1000) {
                    // Set the event for high CO2
                    xEventGroupSetBits(xCO2EventGroup, CO2_HIGH_EVENT);
                } else {
                    // Set the event for low CO2
                    xEventGroupSetBits(xCO2EventGroup, CO2_LOW_EVENT);
                }
            } else {
                printf("CO2 data not ready...\n");
            }

            vTaskDelay(10000 / portTICK_PERIOD_MS);  // Wait 10 seconds before checking CO2 again
        }*/
        while (1) {
            trigger_single_measurement();
            // Read the latest CO2 concentration
            co2_ppm = co2sensor_read_co2();
            printf("CO2 Concentration: %d ppm\n", co2_ppm);

            // Handle CO2 level thresholds (you can modify this as needed)
            if (co2_ppm > 1000) {
                xEventGroupSetBits(xCO2EventGroup, CO2_HIGH_EVENT);  // Set event for high CO2
            } else {
                xEventGroupSetBits(xCO2EventGroup, CO2_LOW_EVENT);  // Set event for low CO2
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay before reading again (for example, 1 second)
        }
    }

    void app_main(void) {
        // Create the event group
        xCO2EventGroup = xEventGroupCreate();

        // Initialize the motor
        motor_init();

        // Create FreeRTOS tasks for motor and CO2 sensor
        xTaskCreate(motor_task, "Motor Task", 2048, NULL, 5, NULL);
        xTaskCreate(co2sensor_task, "CO2 Sensor Task", 2048, NULL, 5, NULL);
    }
