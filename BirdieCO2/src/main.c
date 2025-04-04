#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "esp_wifi.h"

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
#define CALIB_BUTTON_GPIO GPIO_NUM_0
#define CO2_LOW_VALUE 500

static EventGroupHandle_t xCO2EventGroup = NULL;

static const char *TAG_MAIN  = "MAIN";
static const char *TAG_MOTOR = "MOTOR";
static const char *TAG_BATT  = "BATT";

RTC_DATA_ATTR int last_motor_angle = -1;

typedef struct {
    uint16_t co2_median;
    float voltage;
    float soc;
} co2_data_t;

static co2_data_t latest_data;  // Global variable to store the latest data

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
                                               CO2_HIGH_EVENT | CO2_LOW_EVENT,
                                               pdTRUE, pdFALSE, portMAX_DELAY);

        int target_angle = -1;

        if (bits & CO2_HIGH_EVENT){
            target_angle = 90;
        }
        else if (bits & CO2_LOW_EVENT) {
            target_angle = 0;
        }

        if (target_angle >= 0 && target_angle != current_angle) {
            ESP_LOGI(TAG_MOTOR, "Motor angle: %d° → %d°", current_angle, target_angle);
            motor_set_angle(target_angle);
            vTaskDelay(pdMS_TO_TICKS(1000));
            last_motor_angle = current_angle = target_angle;
        } else {
            ESP_LOGI(TAG_MOTOR, "Motor unchanged: %d°", current_angle);
        }
    }
}

static void co2_measure_task(void *pvParameters) {
    ESP_LOGI("CO2", "CO2 measurement task started");

    if (is_fresh_boot()) {
        ESP_LOGI("CO2", "Cold boot detected → soft resetting & calibrating sensor");
        co2sensor_soft_reset();
        vTaskDelay(pdMS_TO_TICKS(2000));
        co2sensor_calibrate(CO2_LOW_VALUE);
        vTaskDelay(pdMS_TO_TICKS(2000));

        ESP_LOGI("CO2", "Taking 6 dummy measurements after calibration");
        for (int i = 0; i < 6; i++) {
            trigger_single_measurement();
            uint16_t dummy;
            co2sensor_read_co2(&dummy);
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
    }

    ESP_LOGI("CO2", "Taking actual measurements...");
    uint16_t readings[6];
    for (int i = 0; i < 6; i++) {
        trigger_single_measurement();
        co2sensor_read_co2(&readings[i]);
        ESP_LOGI("CO2", "Reading %d = %d ppm", i + 1, readings[i]);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    latest_data.co2_median = calculate_median(readings, 6);
    latest_data.voltage = battery_get_voltage();
    latest_data.soc = battery_get_soc();

    ESP_LOGI("CO2", "Median CO2 = %d ppm | Voltage = %.2f V | SOC = %.2f%%",
        latest_data.co2_median, latest_data.voltage, latest_data.soc);

    if (latest_data.soc < 5.0f) {
        ESP_LOGW("CO2", "Low battery, overriding motor to 180°");
        motor_set_angle(180);
        last_motor_angle = 180;
    } else {
        if (latest_data.co2_median > 1000){
            ESP_LOGI("CO2", "High CO2 detected → setting HIGH event");
            xEventGroupSetBits(xCO2EventGroup, CO2_HIGH_EVENT);
        }
        else{
            ESP_LOGI("CO2", "Low CO2 detected → setting LOW event");
            xEventGroupSetBits(xCO2EventGroup, CO2_LOW_EVENT);
        }
    }

    ESP_LOGI("CO2", "Setting CO2_DATA_READY_EVENT");
    xEventGroupSetBits(xCO2EventGroup, CO2_DATA_READY_EVENT);
    vTaskDelete(NULL);
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
    EventBits_t bits = xEventGroupWaitBits(xCO2EventGroup, CO2_DATA_READY_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

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

        vTaskDelay(pdMS_TO_TICKS(600000)); // 10 min
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

    i2c_master_init();
    i2c_scan();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CALIB_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /*if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0 &&
        gpio_get_level(CALIB_BUTTON_GPIO) == 0) {
        ESP_LOGW(TAG_MAIN, "Button wake → calibrating");
        co2sensor_soft_reset();
        vTaskDelay(pdMS_TO_TICKS(50));
        co2sensor_calibrate(CO2_LOW_VALUE);
    }*/

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
    
    if (xTaskCreate(co2_measure_task, "co2Task", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create CO₂ sensor task");
    }

    if (xTaskCreate(mqtt_task, "mqttTask", 4096, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create CO₂ sensor task");
    }
   
    ESP_LOGI(TAG_MAIN, "System initialized");
}