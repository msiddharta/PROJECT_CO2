#include "co2sensor.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


static const char *TAG_CO2 = "CO2_SENSOR";


esp_err_t trigger_single_measurement() {
    uint8_t mode = 0x01;  // Single-shot
    esp_err_t ret = i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &mode, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_CO2, "Failed to trigger measurement");
        return ret;
    }

    for (int i = 0; i < MAX_READY_RETRIES; i++) {
        uint8_t status = 0;
        if (i2c_read_bytes(CO2_ADDR, REG_MEAS_STS, &status, 1) == ESP_OK &&
            (status & DRDY_BIT)) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(MEAS_DELAY_MS));
    }

    ESP_LOGW(TAG_CO2, "Timeout waiting for data ready");
    return ESP_ERR_TIMEOUT;
}

esp_err_t co2sensor_read_co2(uint16_t *ppm_out) {
    if (!ppm_out) return ESP_ERR_INVALID_ARG;

    uint8_t high = 0, low = 0;
    if (i2c_read_bytes(CO2_ADDR, REG_CO2PPM_H, &high, 1) != ESP_OK ||
        i2c_read_bytes(CO2_ADDR, REG_CO2PPM_L, &low, 1) != ESP_OK) {
        ESP_LOGE(TAG_CO2, "Failed to read CO2 register");
        return ESP_FAIL;
    }

    *ppm_out = ((uint16_t)high << 8) | low;
    return ESP_OK;
}

esp_err_t co2sensor_calibrate(uint16_t ref_ppm) {
    if (ref_ppm < 350) ref_ppm = 350;
    if (ref_ppm > 1500) ref_ppm = 1500;

    uint8_t high = (ref_ppm >> 8) & 0xFF;
    uint8_t low  = ref_ppm & 0xFF;

    esp_err_t ret1 = i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_H, &high, 1);
    esp_err_t ret2 = i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_L, &low, 1);
    if (ret1 != ESP_OK || ret2 != ESP_OK) {
        ESP_LOGE(TAG_CO2, "Failed to write calibration value");
        return ESP_FAIL;
    }

    // Debug read-back of calibration value
    uint8_t chk_high = 0, chk_low = 0;
    i2c_read_bytes(CO2_ADDR, REG_CALIB_REF_H, &chk_high, 1);
    i2c_read_bytes(CO2_ADDR, REG_CALIB_REF_L, &chk_low, 1);
    uint16_t actual_ppm = ((uint16_t)chk_high << 8) | chk_low;
    ESP_LOGI(TAG_CO2, "Calib reg check: 0x%02X%02X (%d ppm)", chk_high, chk_low, actual_ppm);


    ESP_LOGI(TAG_CO2, "Calibration set to %d ppm", ref_ppm);

    uint8_t save_cmd = SAVE_CALIB_CMD;
    esp_err_t ret3 = i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &save_cmd, 1);
    if (ret3 == ESP_OK)
        ESP_LOGI(TAG_CO2, "Calibration saved to NVM");
    else
        ESP_LOGE(TAG_CO2, "Failed to save calibration");

    return ret3;
}

esp_err_t perform_forced_compensation() {
    ESP_LOGI(TAG_CO2, "Starting forced compensation...");
    for (int i = 0; i < 3; i++) {
        if (trigger_single_measurement() != ESP_OK) {
            ESP_LOGE(TAG_CO2, "Measurement failed during compensation");
            return ESP_FAIL;
        }
        vTaskDelay(pdMS_TO_TICKS(10000)); // Wait between samples
    }

    uint8_t save_cmd = SAVE_CALIB_CMD;
    esp_err_t ret = i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &save_cmd, 1);
    if (ret == ESP_OK)
        ESP_LOGI(TAG_CO2, "Forced compensation saved");
    else
        ESP_LOGE(TAG_CO2, "Failed to save forced compensation");

    return ret;
}

esp_err_t co2sensor_enable_aboc() {
    uint8_t aboc = ABOC_ENABLE_CMD;
    esp_err_t ret = i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &aboc, 1);
    if (ret == ESP_OK)
        ESP_LOGI(TAG_CO2, "ABOC enabled");
    else
        ESP_LOGE(TAG_CO2, "Failed to enable ABOC");
    return ret;
}

esp_err_t co2sensor_soft_reset() {
    uint8_t cmd = SOFT_RESET_CMD;
    esp_err_t ret = i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
    if (ret == ESP_OK)
        ESP_LOGI(TAG_CO2, "Soft reset triggered");
    else
        ESP_LOGE(TAG_CO2, "Soft reset failed");
    return ret;
}