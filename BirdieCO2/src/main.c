#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "motor.h"
#include "co2sensor.h"
#include "max17048_adv.h"  // The advanced IDF-based code for MAX17048
#include "i2c_bus.h"     // The I2C bus driver
#include "battery.h"     // The battery monitor code

// FreeRTOS event bits
#define CO2_HIGH_EVENT (1 << 0)
#define CO2_LOW_EVENT  (1 << 1)

// We'll store the event group for coordinating motor tasks
static EventGroupHandle_t xCO2EventGroup = NULL;

static const char *TAG_BATT = "BATTERY";
static const char *TAG_MAIN = "MAIN";
static const char *TAG_CO2 = "CO2";
static const char *TAG_MOTOR = "MOTOR";

RTC_DATA_ATTR int last_motor_angle = -1;  // -1 means unknown
// ---------------------------------------------------------------------
// Motor task: waits for CO2 events to set servo angle
// ---------------------------------------------------------------------
static void motor_task(void *pvParameters) {
    int current_angle = last_motor_angle;  // unknown at startup

    while (true)
    {
        // Wait for CO2 events (clear bits on exit)
        EventBits_t bits = xEventGroupWaitBits(
            xCO2EventGroup,
            CO2_HIGH_EVENT | CO2_LOW_EVENT,
            pdTRUE,
            pdFALSE,
            portMAX_DELAY);

        int target_angle = -1;

        if (bits & CO2_HIGH_EVENT) {
            target_angle = 90;
        } 
        else if (bits & CO2_LOW_EVENT) {
            target_angle = 0;
        }

        if (target_angle >= 0 && target_angle != current_angle) {
            ESP_LOGI(TAG_MOTOR, "Changing motor angle: %d° → %d°", current_angle, target_angle);
            motor_set_angle(target_angle);
            last_motor_angle  = target_angle;
        } 
        else {
            ESP_LOGI(TAG_MOTOR, "No motor change needed (angle already %d°)", current_angle);
        }

    }
}

// ---------------------------------------------------------------------
// CO2 sensor task: uses co2sensor_xxx calls
// ---------------------------------------------------------------------
static void co2sensor_task(void *pvParameters)
{
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    bool fresh_boot = (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER);

    if (fresh_boot) {
        ESP_LOGI(TAG_CO2, "Cold boot: running stabilization for 2 minutes");
        for (int i = 0; i < 60; i++) {
            trigger_single_measurement();
            co2sensor_read_co2(); // discard
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
        ESP_LOGI(TAG_CO2, "Stabilization complete");
    } else {
        ESP_LOGI(TAG_CO2, "Wake from deep sleep: skipping stabilization and delay");
    }

    // Do sampling now (always)
    uint16_t readings[6];
    for (int i = 0; i < 6; i++) {
        trigger_single_measurement();
        readings[i] = co2sensor_read_co2();
        ESP_LOGI(TAG_CO2, "Sample %d: %d ppm", i + 1, readings[i]);
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10 sec delay
    }

    // Sort readings (bubble sort)
    for (int i = 0; i < 5; i++) {
        for (int j = i + 1; j < 6; j++) {
            if (readings[i] > readings[j]) {
                uint16_t tmp = readings[i];
                readings[i] = readings[j];
                readings[j] = tmp;
            }
        }
    }

    uint16_t median = (readings[2] + readings[3]) / 2;
    ESP_LOGI(TAG_CO2, "CO2 Median: %d ppm", median);

    if (median > 1000)
        xEventGroupSetBits(xCO2EventGroup, CO2_HIGH_EVENT);
    else
        xEventGroupSetBits(xCO2EventGroup, CO2_LOW_EVENT);

    ESP_LOGI(TAG_CO2, "Preparing to enter deep sleep for 9 minutes...");
    esp_sleep_enable_timer_wakeup(9 * 60 * 1000000ULL);  // 9 min in µs
    vTaskDelay(pdMS_TO_TICKS(200));  // Let logs flush
    esp_deep_sleep_start();  // Never returns
}


// ---------------------------------------------------------------------
// Battery task: uses advanced MAX17048 features
// ---------------------------------------------------------------------
static void battery_task(void *pvParameters)
{
    // Optionally set advanced features
    max17048_set_alert_voltages(3.3f, 4.2f);
    float minv, maxv;
    if (max17048_get_alert_voltages(&minv, &maxv) == ESP_OK) {
        ESP_LOGI(TAG_BATT, "Alert voltages=%.2f ~ %.2f", minv, maxv);
    }

    while (true)
    {
        extern float battery_get_voltage(void);
        extern float battery_get_soc(void);

        float voltage = battery_get_voltage();
        float percent = battery_get_soc();

        if (voltage < 0.0f || percent < 0.0f) {
            ESP_LOGE(TAG_BATT, "Battery read error!");
        } else {
            ESP_LOGI(TAG_BATT, "Voltage=%.3f V, SoC=%.2f %%", voltage, percent);
        }

        // Check if any advanced alerts triggered
        uint8_t flags = max17048_get_alert_flags();
        if (flags) {
            ESP_LOGW(TAG_BATT, "Alert flags=0x%02X", flags);
            // e.g. if (flags & MAX1704X_ALERTFLAG_VOLTAGE_LOW) ...
            max17048_clear_alert_flag(flags); // Clear them all or choose which bits
        }

        vTaskDelay(pdMS_TO_TICKS(10 * 60 * 1000));  // 10 minutes
    }
}

// ---------------------------------------------------------------------
// app_main: main entry point for ESP-IDF application
// This is where we initialize everything and create tasks.
// ---------------------------------------------------------------------
void app_main(void)
{
    // ENABLE STEMMA QT else sda,scl pin low
    // Configure GPIO7 as output and set HIGH
    gpio_set_direction(GPIO_NUM_7, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    // Create event group for motor and co2 sensor
    xCO2EventGroup = xEventGroupCreate();
    if (!xCO2EventGroup) {
        ESP_LOGE(TAG_MAIN, "Failed to create event group!");
        return;
    }

    // Initialize I2C driver
    i2c_master_init();
    i2c_scan(); // optional: see which addresses appear (should see 0x28, maybe 0x36, etc.)

    // Initialize the motor (servo)
    motor_init();

    // Initialize the CO2 sensor (Calibration, ABOC, etc.)	
    co2sensor_init(330);  // 330 ppm reference for calibration

    // Enable timer wakeup for deep sleep (in microseconds)
    esp_sleep_enable_timer_wakeup(9 * 60 * 1000000ULL);  // 9 minutes

    // Check if MAX17048 is present
    esp_err_t ret = max17048_init_check();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG_MAIN, "MAX17048 found!");
        if (xTaskCreate(battery_task, "batteryTask", 4096, NULL, 5, NULL) != pdPASS) {
            ESP_LOGE(TAG_MAIN, "Failed to create motorTask");
        }
    } else {
        ESP_LOGW(TAG_MAIN, "MAX17048 not found or no battery connected!");
        // either skip creating battery task or create a minimal one
    }

    // 6) Create tasks
    if (xTaskCreate(motor_task, "motorTask", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create motorTask");
    }
    if (xTaskCreate(co2sensor_task, "co2Task", 2048, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG_MAIN, "Failed to create co2Task");
    }
    ESP_LOGI(TAG_MAIN, "app_main done!");
}