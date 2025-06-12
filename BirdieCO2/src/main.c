#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include "motor.h"         // Servo motor control functions
#include "co2sensor.h"     // CO₂ sensor functions
#include "battery.h"       // Battery monitoring functions
#include "max17048_adv.h"  // Advanced battery sensor functions for MAX17048
#include "i2c_bus.h"       // I2C communication functions
#include "mqtt.h"          // MQTT and Wi-Fi management functions
#include "sensor_state.h"  // Encapsulated sensor state
#include "app_config.h"    // Central configuration constants

// Global FreeRTOS objects for event signaling and mutual exclusion.
static EventGroupHandle_t xCO2EventGroup = NULL;
static TaskHandle_t xCO2MeasureHandle = NULL;
static SemaphoreHandle_t xSensorMutex = NULL;

// Encapsulated shared sensor state stored in RTC memory.
RTC_DATA_ATTR sensor_state_t sensor_state = {
    .last_motor_angle = -1,
    .last_co2_value = 0,
    .last_button_press_us = 0,
    .stop_measurement = false
};

// Define consistent logging tags for all modules.
static const char *TAG_MAIN   = "MAIN";
static const char *TAG_MOTOR  = "MOTOR";
static const char *TAG_BATT   = "BATT";
static const char *TAG_CO2    = "CO2";
static const char *TAG_MQTT   = "MQTT";
static const char *TAG_CALIB  = "CALIB";

typedef struct {
    uint16_t co2_median;
    float voltage;
    float soc;
} co2_data_t;

static co2_data_t latest_data;  // Stores the latest sensor readings.

static void co2_measure_task(void *pvParameters);

/**
 * @brief Calibration button ISR.
 *
 * Debounces the calibration button using the sensor_state timestamp and signals
 * the calibration task by setting the EVENT_CO2_RECALIBRATE bit.
 */
static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t now = esp_timer_get_time();  // Current time in microseconds
    if (now - sensor_state.last_button_press_us < BUTTON_DEBOUNCE_TIME_US)
        return;  // Debounce: ignore if less than BUTTON_DEBOUNCE_TIME_US has elapsed.
    sensor_state.last_button_press_us = now;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(xCO2EventGroup, EVENT_CO2_RECALIBRATE, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Determine if the system woke up due to a fresh boot.
 *
 * @return true if the wakeup cause is not a timer (deep sleep) wakeup.
 */
static inline bool is_fresh_boot() {
    return esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER;
}

/**
 * @brief Calculate the median for an array of 16-bit sensor readings.
 *
 * Uses a simple bubble sort given the small array size.
 *
 * @param data Array of readings.
 * @param size Number of readings.
 * @return uint16_t The computed median value.
 */
static uint16_t calculate_median(uint16_t *data, int size) {
    for (int i = 0; i < size - 1; i++) {
        for (int j = i + 1; j < size; j++) {
            if (data[i] > data[j]) {
                uint16_t tmp = data[i];
                data[i] = data[j];
                data[j] = tmp;
            }
        }
    }
    return (data[(size - 1) / 2] + data[size / 2]) / 2;
}

/**
 * @brief Task for performing sensor calibration when triggered.
 *
 * Waits for the EVENT_CO2_RECALIBRATE, halts any running measurement task,
 * acquires the sensor mutex, executes calibration steps, and restarts measurement.
 */
static void calibration_task(void *pvParameters) {
    while (true) {
        EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup, EVENT_CO2_RECALIBRATE, pdTRUE, pdFALSE, portMAX_DELAY);
        if (bits & EVENT_CO2_RECALIBRATE) {
            ESP_LOGW(TAG_CALIB, "Calibration triggered by button!");

            if (xCO2MeasureHandle != NULL) {
                ESP_LOGW(TAG_CALIB, "Stopping running CO2 measurement task");
                sensor_state.stop_measurement = true;
                vTaskDelay(pdMS_TO_TICKS(TIME_SHORT_DELAY_MS));
                xCO2MeasureHandle = NULL;
            }

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) != pdTRUE) {
                ESP_LOGE(TAG_CALIB, "Could not acquire sensor mutex");
                continue;
            }

            if (co2sensor_wait_for_ready(TIME_WAIT_FOR_READYNESS_MS) != ESP_OK) {
                ESP_LOGE(TAG_CALIB, "Sensor not ready after reset!");
                xSemaphoreGive(xSensorMutex);
                continue;
            }
            vTaskDelay(pdMS_TO_TICKS(SENSOR_INIT_WAIT_TIME_MS));

            if (co2sensor_set_pressure(SENSOR_PRESSURE_HPA) != ESP_OK)
                ESP_LOGE(TAG_CALIB, "Set pressure FAILED");
            else
                ESP_LOGI(TAG_CALIB, "Set pressure OK");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

            if (co2sensor_disable_aboc() != ESP_OK)
                ESP_LOGE(TAG_CALIB, "Disable ABOC FAILED");
            else
                ESP_LOGI(TAG_CALIB, "Disable ABOC OK");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

            if (co2sensor_disable_iir() != ESP_OK)
                ESP_LOGE(TAG_CALIB, "Disable IIR FAILED");
            else
                ESP_LOGI(TAG_CALIB, "Disable IIR OK");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

            if (co2sensor_calibrate(CO2_LOW_THRESHOLD) != ESP_OK)
                ESP_LOGE(TAG_CALIB, "Calibration set FAILED");
            else
                ESP_LOGI(TAG_CALIB, "Calibration set OK");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

            if (co2sensor_force_compensation() != ESP_OK)
                ESP_LOGE(TAG_CALIB, "Forced Compensation FAILED");
            else
                ESP_LOGI(TAG_CALIB, "Forced Compensation OK");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

            if (co2sensor_disable_aboc() != ESP_OK)
                ESP_LOGE(TAG_CALIB, "Final disable ABOC FAILED");
            else
                ESP_LOGI(TAG_CALIB, "Final disable ABOC OK");
            vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

            ESP_LOGI(TAG_CALIB, "Calibration complete, restarting measurement task");
            xSemaphoreGive(xSensorMutex);
            xTaskCreate(co2_measure_task, "co2Task", CO2_MEASURE_TASK_STACK_SIZE, NULL, 5, &xCO2MeasureHandle);
        }
    }
}

/**
 * @brief Task to control the servo motor.
 *
 * Initializes the motor, performs a cold boot test if necessary, restores the last motor angle,
 * and then adjusts the motor angle based on battery and CO₂ event signals.
 */
static void motor_task(void *pvParameters) {
    bool cold_boot = is_fresh_boot();
    motor_init();

    if (cold_boot && sensor_state.last_motor_angle == MOTOR_ANGLE_UNDEFINED) {
        ESP_LOGI(TAG_MOTOR, "Motor test (cold boot): %d° -> %d°", MOTOR_ANGLE_HIGH, MOTOR_ANGLE_LOW);
        motor_set_angle(MOTOR_ANGLE_HIGH);
        vTaskDelay(pdMS_TO_TICKS(TIME_SHORT_DELAY_MS));
        motor_set_angle(MOTOR_ANGLE_LOW);
        vTaskDelay(pdMS_TO_TICKS(TIME_SHORT_DELAY_MS));
        sensor_state.last_motor_angle = MOTOR_ANGLE_LOW;
    } else if (sensor_state.last_motor_angle <= MOTOR_ANGLE_LOW) { //for assembly purposes, else >=
        ESP_LOGI(TAG_MOTOR, "Restoring motor angle: %d°", sensor_state.last_motor_angle);
        motor_set_angle(sensor_state.last_motor_angle);
    }

    int current_angle = sensor_state.last_motor_angle;

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup,
                                               EVENT_BATT_LOW | EVENT_CO2_HIGH | EVENT_CO2_LOW,
                                               pdTRUE, pdFALSE, portMAX_DELAY);

        int target_angle = -1;
        if (bits & EVENT_BATT_LOW) {
            target_angle = MOTOR_ANGLE_HIGH;
        } else if (bits & EVENT_CO2_HIGH) {
            target_angle = MOTOR_ANGLE_MED;
        } else if (bits & EVENT_CO2_LOW) {
            target_angle = MOTOR_ANGLE_LOW;
        }
    
        if (target_angle != sensor_state.last_motor_angle) {
            ESP_LOGI(TAG_MOTOR, "Motor angle: %d° -> %d°, Last motor angle: %d", current_angle, target_angle, sensor_state.last_motor_angle);
            sensor_state.last_motor_angle = target_angle;
            motor_set_angle(target_angle);
        } else {
            ESP_LOGI(TAG_MOTOR, "Motor unchanged: %d°", sensor_state.last_motor_angle);
        }
    }
}

/**
 * @brief CO₂ measurement task.
 *
 * Acquires the sensor mutex, initializes the CO₂ sensor on cold boot,
 * takes multiple measurements, computes the median CO₂ reading, reads battery data,
 * validates large CO₂ jumps, and sets the corresponding event flags.
 */
static void co2_measure_task(void *pvParameters) {
    ESP_LOGI(TAG_CO2, "CO₂ measurement task started");

    // Reset the stop flag to ensure fresh start 
    sensor_state.stop_measurement = false;

    if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG_CO2, "Could not acquire sensor mutex");
        vTaskDelete(NULL);
    }

    if (is_fresh_boot()) {
        ESP_LOGI(TAG_CO2, "Cold boot detected → sensor initialization started");
        if (co2sensor_soft_reset() != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Soft reset FAILED");
        } else {
            ESP_LOGI(TAG_CO2, "Soft reset OK");
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INIT_WAIT_TIME_MS));

        if (co2sensor_set_pressure(SENSOR_PRESSURE_HPA) != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Set pressure FAILED");
        } else {
            ESP_LOGI(TAG_CO2, "Set pressure OK");
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));

        if (co2sensor_disable_aboc() != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Disable ABOC FAILED");
        } else {
            ESP_LOGI(TAG_CO2, "Disable ABOC OK");
        }
        if (co2sensor_disable_iir() != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Disable IIR filter FAILED");
        } else {
            ESP_LOGI(TAG_CO2, "Disable IIR filter OK");
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_STABILIZATION_TIME_MS));
        ESP_LOGI(TAG_CO2, "Initialization completed successfully");
    }

    ESP_LOGI(TAG_CO2, "Starting measurements...");
    uint16_t readings[6];
    for (int i = 0; i < 6; i++) {
        if (sensor_state.stop_measurement) {
            ESP_LOGW(TAG_CO2, "Measurement task interrupted by calibration");
            sensor_state.stop_measurement = false;
            xSemaphoreGive(xSensorMutex);
            vTaskDelete(NULL);
        }

        if (co2sensor_trigger_measurement() != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Trigger measurement %d FAILED", i + 1);
            readings[i] = 0xFFFF;
        } else if (co2sensor_read_ppm(&readings[i]) != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Read PPM measurement %d FAILED", i + 1);
            readings[i] = 0xFFFF;
        } else {
            ESP_LOGI(TAG_CO2, "Reading %d = %d ppm", i + 1, readings[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(MEASUREMENT_INTERVAL_MS));
    }

    latest_data.co2_median = calculate_median(readings, 6);
    latest_data.voltage = battery_get_voltage();
    latest_data.soc = battery_get_soc();

    ESP_LOGI(TAG_CO2, "Median CO₂ = %d ppm | Voltage = %.2f V | SOC = %.2f%%",
             latest_data.co2_median, latest_data.voltage, latest_data.soc);

    if (!is_fresh_boot()) {
        uint16_t diff = abs((int)latest_data.co2_median - (int)sensor_state.last_co2_value);
        if (diff >= CO2_DIFF_THRESHOLD) {
            ESP_LOGW(TAG_CO2, "CO₂ jumped by %d ppm → verifying with 2nd measurement", diff);
            uint16_t readings2[3];
            for (int i = 0; i < 3; i++) {
                if (sensor_state.stop_measurement) {
                    ESP_LOGW(TAG_CO2, "Measurement task interrupted by calibration");
                    sensor_state.stop_measurement = false;
                    xSemaphoreGive(xSensorMutex);
                    vTaskDelete(NULL);
                }

                if (co2sensor_trigger_measurement() != ESP_OK) {
                    ESP_LOGE(TAG_CO2, "2nd trigger measurement %d FAILED", i + 1);
                    readings2[i] = 0xFFFF;
                } else if (co2sensor_read_ppm(&readings2[i]) != ESP_OK) {
                    ESP_LOGE(TAG_CO2, "2nd read PPM measurement %d FAILED", i + 1);
                    readings2[i] = 0xFFFF;
                } else {
                    ESP_LOGI(TAG_CO2, "2nd Reading %d = %d ppm", i + 1, readings2[i]);
                }
                vTaskDelay(pdMS_TO_TICKS(TIME_SHORT_DELAY_MS * 2)); // 2-second delay between verification readings
            }
            latest_data.co2_median = calculate_median(readings2, 3);
            ESP_LOGI(TAG_CO2, "Median2 CO₂ = %d ppm (after large jump)", latest_data.co2_median);
        }
    }

    sensor_state.last_co2_value = latest_data.co2_median;

    if (latest_data.soc < 10.0f) {
        ESP_LOGW(TAG_CO2, "Low battery => set BATT_LOW_EVENT");
        xEventGroupSetBits(xCO2EventGroup, EVENT_BATT_LOW);
    } else if (latest_data.co2_median > CO2_HIGH_THRESHOLD) {
        ESP_LOGI(TAG_CO2, "High CO₂ => set CO2_HIGH_EVENT");
        xEventGroupSetBits(xCO2EventGroup, EVENT_CO2_HIGH);
    } else {
        ESP_LOGI(TAG_CO2, "CO₂ normal => set CO2_LOW_EVENT");
        xEventGroupSetBits(xCO2EventGroup, EVENT_CO2_LOW);
    }

    ESP_LOGI(TAG_CO2, "Measurement task done — setting CO2_DATA_READY_EVENT");
    xEventGroupSetBits(xCO2EventGroup, EVENT_CO2_DATA_READY);

    xSemaphoreGive(xSensorMutex);
    if (xCO2MeasureHandle != NULL) {
        ESP_LOGW(TAG_CO2, "Stopping running CO₂ measurement task");
        sensor_state.stop_measurement = true;
        vTaskDelay(pdMS_TO_TICKS(TIME_SHORTEST_DELAY_MS));
        xCO2MeasureHandle = NULL;
    }

    // Explicitly delete the task since it is one-shot.
    vTaskDelete(NULL);
}

/**
 * @brief MQTT communication task.
 *
 * Initializes Wi-Fi and MQTT, waits for CO₂ data readiness, publishes sensor data via MQTT,
 * then stops Wi-Fi/MQTT and puts the system into deep sleep.
 */
static void mqtt_task(void *pvParameters) {
    ESP_LOGI(TAG_MQTT, "Starting MQTT task");
    wifi_init_sta();
    mqtt_start();
    vTaskDelay(pdMS_TO_TICKS(TIME_SHORT_DELAY_MS));  // Wait for MQTT to be ready.

    if (is_fresh_boot()) {
        ESP_LOGI(TAG_MQTT, "Cold boot → sending MQTT discovery message");
        mqtt_discovery_publish();
    }

    ESP_LOGI(TAG_MQTT, "Waiting for CO2_DATA_READY_EVENT...");
    EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup, EVENT_CO2_DATA_READY, pdTRUE, pdFALSE, pdMS_TO_TICKS(TIME_WAIT_FOR_SENSOR_MS));
    if (bits & EVENT_CO2_DATA_READY) {
        publish_data(latest_data.co2_median, latest_data.voltage, latest_data.soc);
        ESP_LOGI(TAG_MQTT, "Publishing: CO₂=%d ppm, V=%.2f, SOC=%.2f",
                 latest_data.co2_median, latest_data.voltage, latest_data.soc);
    } else {
        ESP_LOGW(TAG_MQTT, "Timeout waiting for CO₂ data — not publishing");
    }

    mqtt_stop();
    esp_wifi_stop();

    ESP_LOGI(TAG_MQTT, "Entering deep sleep for 10 minutes");
    esp_sleep_enable_timer_wakeup(DEEP_SLEEP_TIME_US);

    gpio_set_level(LED_GPIO, GPIO_LOW);  // Turn off LED.
    vTaskDelay(pdMS_TO_TICKS(TIME_SHORTEST_DELAY_MS));
    esp_deep_sleep_start();
}

/**
 * @brief Battery monitoring task.
 *
 * Sets battery alert voltage thresholds, periodically reads battery voltage and SOC,
 * logs the results, and clears any alert flags.
 */
static void battery_task(void *pvParameters) {
    max17048_set_alert_voltages(BATTERY_ALERT_MIN_VOLTAGE, BATTERY_ALERT_MAX_VOLTAGE);
    float minv, maxv;
    if (max17048_get_alert_voltages(&minv, &maxv) == ESP_OK)
        ESP_LOGI(TAG_BATT, "Alert voltages = %.2f ~ %.2f", minv, maxv);

    while (true) {
        float voltage = battery_get_voltage();
        float percent = battery_get_soc();
        if (voltage < 0.0f || percent < 0.0f)
            ESP_LOGE(TAG_BATT, "Battery read error!");
        else
            ESP_LOGI(TAG_BATT, "Battery: %.3f V, %.2f %%", voltage, percent);

        uint8_t flags = max17048_get_alert_flags();
        if (flags) {
            ESP_LOGW(TAG_BATT, "Alert flags = 0x%02X", flags);
            max17048_clear_alert_flag(flags);
        }
        vTaskDelay(pdMS_TO_TICKS(BATTERY_CHECK_INTERVAL_MS));
    }
}

/**
 * @brief Main application entry point.
 *
 * Sets up hardware resources (I2C, GPIOs), creates FreeRTOS objects,
 * installs the calibration button ISR, and creates tasks for battery monitoring,
 * motor control, CO₂ measurement, MQTT communication, and calibration.
 */
void app_main(void) {
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, GPIO_HIGH);

    xCO2EventGroup = xEventGroupCreate();
    if (!xCO2EventGroup) {
        ESP_LOGE(TAG_MAIN, "Event group creation failed");
        return;
    }

    xSensorMutex = xSemaphoreCreateMutex();
    if (xSensorMutex == NULL) {
        ESP_LOGE(TAG_MAIN, "Sensor mutex creation failed");
        return;
    }

    esp_log_level_set("wifi", ESP_LOG_WARN);
    i2c_master_init();
    i2c_scan();

    // Configure calibration button GPIO with falling edge interrupt.
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CALIB_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, GPIO_HIGH);  // Turn LED on

    vTaskDelay(pdMS_TO_TICKS(SENSOR_INIT_WAIT_TIME_MS));

    ESP_LOGI(TAG_MAIN, "Wakeup cause = %d, last_motor_angle=%d", esp_sleep_get_wakeup_cause(), sensor_state.last_motor_angle);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(CALIB_BUTTON_GPIO, button_isr_handler, NULL);

    if (max17048_init_check() == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "MAX17048 found");
        if (xTaskCreate(battery_task, "batteryTask", BATTERY_TASK_STACK_SIZE, NULL, 2, NULL) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Failed to create battery task");
        }
    } else {
        ESP_LOGW(TAG_MAIN, "Battery monitor not found");
    }

    if (xTaskCreate(motor_task, "motorTask", MOTOR_TASK_STACK_SIZE, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create motor task");
    }
    
    if (xTaskCreate(co2_measure_task, "co2Task", CO2_MEASURE_TASK_STACK_SIZE, NULL, 5, &xCO2MeasureHandle) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create CO₂ sensor task");
    }

    if (xTaskCreate(mqtt_task, "mqttTask", MQTT_TASK_STACK_SIZE, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create MQTT task");
    }
   
    if (xTaskCreate(calibration_task, "calibrationTask", CO2_CALIB_TASK_STACK_SIZE, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create calibration task");
    }
    
    ESP_LOGI(TAG_MAIN, "System initialized");
}
