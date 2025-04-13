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

#include "motor.h"
#include "co2sensor.h"
#include "battery.h"
#include "max17048_adv.h"
#include "i2c_bus.h"
#include "mqtt.h"

#define CO2_HIGH_EVENT (1 << 0)
#define CO2_LOW_EVENT  (1 << 1)
#define CO2_RECALIBRATE_EVENT (1 << 2)
#define CO2_DATA_READY_EVENT  (1 << 3)
#define BATT_LOW_EVENT  (1 << 4)
#define CALIB_BUTTON_GPIO GPIO_NUM_0
#define LED_GPIO GPIO_NUM_13
#define CO2_LOW_VALUE 400
#define PRESSURE_HPA 1017

static EventGroupHandle_t xCO2EventGroup = NULL;
static TaskHandle_t xCO2MeasureHandle = NULL;
static SemaphoreHandle_t xSensorMutex = NULL;

static volatile bool stop_measurement = false;

static const char *TAG_MAIN  = "MAIN";
static const char *TAG_MOTOR = "MOTOR";
static const char *TAG_BATT  = "BATT";

RTC_DATA_ATTR int last_motor_angle = -1;
RTC_DATA_ATTR static uint16_t last_co2_value = 0;
static volatile int64_t last_button_press_us = 0;

typedef struct {
    uint16_t co2_median;
    float voltage;
    float soc;
} co2_data_t;

static co2_data_t latest_data;  // Global variable to store the latest data

static void co2_measure_task(void *pvParameters);

static void IRAM_ATTR button_isr_handler(void* arg) {
    int64_t now = esp_timer_get_time();  // ma since Boot
    if (now - last_button_press_us < 300000)
        return;  // 300ms debouncing
    last_button_press_us = now;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(xCO2EventGroup, CO2_RECALIBRATE_EVENT, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static inline bool is_fresh_boot() {
    return esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_TIMER;
}

static uint16_t calculate_median(uint16_t *data, int size) {
    for (int i = 0; i < size - 1; i++)
        for (int j = i + 1; j < size; j++)
            if (data[i] > data[j]) {
                uint16_t tmp = data[i];
                data[i] = data[j];
                data[j] = tmp;
            }
    return (data[(size - 1) / 2] + data[size / 2]) / 2;
}


static void calibration_task(void *pvParameters) {
    while (true) {
        EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup, CO2_RECALIBRATE_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

        if (bits & CO2_RECALIBRATE_EVENT) {
            ESP_LOGW("CALIB", "Calibration triggered by button!");

            if (xCO2MeasureHandle != NULL) {
                ESP_LOGW("CALIB", "Stopping running CO2 measurement task");
                stop_measurement = true;
                vTaskDelay(pdMS_TO_TICKS(500));  // Give it time to exit
                xCO2MeasureHandle = NULL; // Let the task self-delete   
            }

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
                ESP_LOGE("CALIB", "Could not acquire sensor mutex");
                continue;
            }

            // Wait until seosr is ready
            if (co2sensor_wait_for_ready(5000) != ESP_OK) {
                ESP_LOGE("CALIB", "Sensor not ready after reset!");
                xSemaphoreGive(xSensorMutex);
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(2000));

            if (co2sensor_set_pressure(PRESSURE_HPA) != ESP_OK)
                ESP_LOGE("CALIB", "Set pressure FAILED");
            else
                ESP_LOGI("CALIB", "Set pressure OK");
            vTaskDelay(pdMS_TO_TICKS(100));

            if (co2sensor_disable_aboc() != ESP_OK)
                ESP_LOGE("CALIB", "Disable ABOC FAILED");
            else
                ESP_LOGI("CALIB", "Disable ABOC OK");
            vTaskDelay(pdMS_TO_TICKS(100));

            if (co2sensor_disable_iir() != ESP_OK)
                ESP_LOGE("CALIB", "Disable IIR FAILED");
            else
                ESP_LOGI("CALIB", "Disable IIR OK");
            vTaskDelay(pdMS_TO_TICKS(100));

            if (co2sensor_calibrate(CO2_LOW_VALUE) != ESP_OK)
                ESP_LOGE("CALIB", "Calibration set FAILED");
            else
                ESP_LOGI("CALIB", "Calibration set OK");
            vTaskDelay(pdMS_TO_TICKS(100));

            if (co2sensor_force_compensation() != ESP_OK)
                ESP_LOGE("CALIB", "Forced Compensation FAILED");
            else
                ESP_LOGI("CALIB", "Forced Compensation OK");
            vTaskDelay(pdMS_TO_TICKS(100));

            if (co2sensor_disable_aboc() != ESP_OK)
                ESP_LOGE("CALIB", "Final disable ABOC FAILED");
            else
                ESP_LOGI("CALIB", "Final disable ABOC OK");
            vTaskDelay(pdMS_TO_TICKS(100));

            ESP_LOGI("CALIB", "Calibration complete, restarting measurement task");
            xSemaphoreGive(xSensorMutex);
            xTaskCreate(co2_measure_task, "co2Task", 4096, NULL, 5, &xCO2MeasureHandle);
        }
    }
}


static void motor_task(void *pvParameters) {
    bool cold_boot = is_fresh_boot();
    motor_init();

    if (cold_boot && last_motor_angle == -1) {
        ESP_LOGI(TAG_MOTOR, "Motor test (cold boot): 180° -> 0°");
        motor_set_angle(180);
        vTaskDelay(pdMS_TO_TICKS(1000));
        motor_set_angle(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        last_motor_angle = 0;
    } else if (last_motor_angle >= 0) {
        ESP_LOGI(TAG_MOTOR, "Restoring motor angle: %d°", last_motor_angle);
        motor_set_angle(last_motor_angle);
    }

    int current_angle = last_motor_angle;

    while (true) {
        EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup,
                                               BATT_LOW_EVENT | CO2_HIGH_EVENT | CO2_LOW_EVENT,
                                               pdTRUE, pdFALSE, portMAX_DELAY);

        int target_angle = -1;

        if (bits & BATT_LOW_EVENT) {
            target_angle = 180;
        } 
        else if (bits & CO2_HIGH_EVENT) {
            target_angle = 90;
        } 
        else if (bits & CO2_LOW_EVENT) {
            target_angle = 0;
        }
    
        if (target_angle != last_motor_angle) {
            ESP_LOGI(TAG_MOTOR, "Motor angle: %d° → %d°, Last motor angle: %d", current_angle, target_angle, last_motor_angle);
            last_motor_angle = target_angle;
            motor_set_angle(target_angle);
        } else {
            ESP_LOGI(TAG_MOTOR, "Motor unchanged: %d°", last_motor_angle);
        }
    }
}

static void co2_measure_task(void *pvParameters) {
    ESP_LOGI("CO2", "CO2 measurement task started");

    if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10000)) != pdTRUE) {
        ESP_LOGE("CO2", "Could not acquire sensor mutex");
        vTaskDelete(NULL);
    }

    if (is_fresh_boot()) {
        ESP_LOGI("CO2", "Cold boot detected → sensor initialization started");

        if (co2sensor_soft_reset() != ESP_OK) {
            ESP_LOGE("CO2", "Soft reset FAILED");
        } else {
            ESP_LOGI("CO2", "Soft reset OK");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));

        if (co2sensor_set_pressure(PRESSURE_HPA) != ESP_OK) {
            ESP_LOGE("CO2", "Set pressure FAILED");
        } else {
            ESP_LOGI("CO2", "Set pressure OK");
        }
        vTaskDelay(pdMS_TO_TICKS(200));

        if (co2sensor_disable_aboc() != ESP_OK) {
            ESP_LOGE("CO2", "Disable ABOC FAILED");
        } else {
            ESP_LOGI("CO2", "Disable ABOC OK");
        }

        if (co2sensor_disable_iir() != ESP_OK) {
            ESP_LOGE("CO2", "Disable IIR filter FAILED");
        } else {
            ESP_LOGI("CO2", "Disable IIR filter OK");
        }
        vTaskDelay(pdMS_TO_TICKS(200));

        ESP_LOGI("CO2", "Initialization completed successfully");
    }

    ESP_LOGI("CO2", "Starting measurements...");
    uint16_t readings[6];
    for (int i = 0; i < 6; i++) {
        if (stop_measurement) {
            ESP_LOGW("CO2", "Measurement task interrupted by calibration");
            stop_measurement = false;
            xSemaphoreGive(xSensorMutex);
            vTaskDelete(NULL);
        }

        if (co2sensor_trigger_measurement() != ESP_OK) {
            ESP_LOGE("CO2", "Trigger measurement %d FAILED", i + 1);
            readings[i] = 0xFFFF; // set error value
        } else if (co2sensor_read_ppm(&readings[i]) != ESP_OK) {
            ESP_LOGE("CO2", "Read PPM measurement %d FAILED", i + 1);
            readings[i] = 0xFFFF; // set error value
        } else {
            ESP_LOGI("CO2", "Reading %d = %d ppm", i + 1, readings[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    latest_data.co2_median = calculate_median(readings, 6);
    latest_data.voltage = battery_get_voltage();
    latest_data.soc = battery_get_soc();

    ESP_LOGI("CO2", "Median CO2 = %d ppm | Voltage = %.2f V | SOC = %.2f%%",
             latest_data.co2_median, latest_data.voltage, latest_data.soc);

    if (!is_fresh_boot()) {
        uint16_t diff = abs((int)latest_data.co2_median - (int)last_co2_value);

        if (diff >= 500) {
            ESP_LOGW("CO2", "CO2 jumped by %d ppm → verifying with 2nd measurement", diff);

            uint16_t readings2[3];
            for (int i = 0; i < 3; i++) {
                if (stop_measurement) {
                    ESP_LOGW("CO2", "Measurement task interrupted by calibration");
                    stop_measurement = false;
                    xSemaphoreGive(xSensorMutex);
                    vTaskDelete(NULL);
                }

                if (co2sensor_trigger_measurement() != ESP_OK) {
                    ESP_LOGE("CO2", "2nd trigger measurement %d FAILED", i + 1);
                    readings2[i] = 0xFFFF;
                } else if (co2sensor_read_ppm(&readings2[i]) != ESP_OK) {
                    ESP_LOGE("CO2", "2nd read PPM measurement %d FAILED", i + 1);
                    readings2[i] = 0xFFFF;
                } else {
                    ESP_LOGI("CO2", "2nd Reading %d = %d ppm", i + 1, readings2[i]);
                }
                vTaskDelay(pdMS_TO_TICKS(2000));
            }
            latest_data.co2_median = calculate_median(readings2, 3);
            ESP_LOGI("CO2", "Median2 CO2 = %d ppm (after large jump)", latest_data.co2_median);
        }
    }

    last_co2_value = latest_data.co2_median;

    if (latest_data.soc < 10.0f) {
        ESP_LOGW("CO2", "Low battery => set BATT_LOW_EVENT");
        xEventGroupSetBits(xCO2EventGroup, BATT_LOW_EVENT);
    } else if (latest_data.co2_median > 1000) {
        ESP_LOGI("CO2", "High CO2 => set CO2_HIGH_EVENT");
        xEventGroupSetBits(xCO2EventGroup, CO2_HIGH_EVENT);
    } else {
        ESP_LOGI("CO2", "CO2 normal => set CO2_LOW_EVENT");
        xEventGroupSetBits(xCO2EventGroup, CO2_LOW_EVENT);
    }

    ESP_LOGI("CO2", "Measurement task done — setting CO2_DATA_READY_EVENT");
    xEventGroupSetBits(xCO2EventGroup, CO2_DATA_READY_EVENT);

    xSemaphoreGive(xSensorMutex);
    if (xCO2MeasureHandle != NULL) {
        ESP_LOGW("CALIB", "Stopping running CO2 measurement task");
        stop_measurement = true;
        vTaskDelay(pdMS_TO_TICKS(300));  // Give time for task to exit
        xCO2MeasureHandle = NULL;        // Let the task self-delete
    }
}


static void mqtt_task(void *pvParameters) {
    ESP_LOGI("MQTT", "Starting MQTT task");
    wifi_init_sta();
    mqtt_start();
    vTaskDelay(pdMS_TO_TICKS(1000));  // until MQTT ready

    if (is_fresh_boot()) {
        ESP_LOGI("MQTT", "Cold boot → sending MQTT discovery message");
        mqtt_discovery_publish();
    }

    ESP_LOGI("MQTT", "Waiting for CO2_DATA_READY_EVENT...");
    EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup, CO2_DATA_READY_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(180000));

    if (bits & CO2_DATA_READY_EVENT) {
        publish_data(latest_data.co2_median, latest_data.voltage, latest_data.soc);
        ESP_LOGI("MQTT", "Publishing: CO2=%d ppm, V=%.2f, SOC=%.2f", latest_data.co2_median, latest_data.voltage, latest_data.soc);
    }

    else{
        ESP_LOGW("MQTT", "Timeout waiting for CO2 data — not publishing");
    }

    mqtt_stop();
    esp_wifi_stop();

    ESP_LOGI("MQTT", "Entering deep sleep for 9 minutes");
    esp_sleep_enable_timer_wakeup(9 * 60 * 1000000ULL);

    gpio_set_level(LED_GPIO, 0);  // LED off

    vTaskDelay(pdMS_TO_TICKS(200));
    esp_deep_sleep_start();
}

static void battery_task(void *pvParameters) {
    max17048_set_alert_voltages(3.3f, 4.2f);
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

        vTaskDelay(pdMS_TO_TICKS(10 * 60 * 1000000ULL)); // 10 min
    }
}

void app_main(void) {
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);

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

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CALIB_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,                   // internal Pull-Up
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE,   // LOW → pressed
    };
    gpio_config(&io_conf);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 1);  // LED on

    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG_MAIN, "Wakeup cause = %d, last_motor_angle=%d", esp_sleep_get_wakeup_cause(), last_motor_angle);

    // Interrupt installieren
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CALIB_BUTTON_GPIO, button_isr_handler, NULL);

    if (max17048_init_check() == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "MAX17048 found");
        if (xTaskCreate(battery_task, "batteryTask", 4096, NULL, 2, NULL) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Failed to create battery task");
        }
    } else {
        ESP_LOGW(TAG_MAIN, "Battery monitor not found");
    }

    if (xTaskCreate(motor_task, "motorTask", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create motor task");
    }
    
    if (xTaskCreate(co2_measure_task, "co2Task", 4096, NULL, 5, &xCO2MeasureHandle) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create CO₂ sensor task");
    }

    if (xTaskCreate(mqtt_task, "mqttTask", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create CO₂ sensor task");
    }
   
    if (xTaskCreate(calibration_task, "calibrationTask", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create calibration task");
    }
    
    ESP_LOGI(TAG_MAIN, "System initialized");
}