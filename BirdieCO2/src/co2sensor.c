#include "co2sensor.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG_CO2 = "CO2_SENSOR";

esp_err_t co2sensor_trigger_measurement() {
    uint8_t cfg;
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    cfg &= ~0x03;               // Clear OP_MODE bits (1:0)
    cfg |= 0x01;                // Set to single-shot mode (0b01)
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

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

esp_err_t co2sensor_read_ppm(uint16_t *ppm_out) {
    if (!ppm_out) return ESP_ERR_INVALID_ARG;
    uint8_t high = 0, low = 0;
    if (i2c_read_bytes(CO2_ADDR, REG_CO2PPM_H, &high, 1) != ESP_OK ||
        i2c_read_bytes(CO2_ADDR, REG_CO2PPM_L, &low, 1) != ESP_OK) {
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
    if (i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_H, &high, 1) != ESP_OK ||
        i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_L, &low, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    uint8_t cmd = SAVE_CALIB_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

esp_err_t co2sensor_force_compensation() {
    uint8_t cfg;
    ESP_LOGW("CALIB", "Forced compensation started!");
    //Set Forced Compensation Bit(BOC_CFG = 10b)
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    cfg = (cfg & ~BOC_CFG_MASK) | BOC_CFG_FORCED; // BOC_CFG_FORCED (0x08)
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    vTaskDelay(pdMS_TO_TICKS(200));

    // Do 3 Measurements to force the compensation
    for (int i = 0; i < 3; i++) {
        if (co2sensor_trigger_measurement() != ESP_OK)
            return ESP_FAIL;
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGW("CALIB", "Forced compensation measurement %d", i + 1);
    }

    // Save the forced compensation in the non volatile memory
    uint8_t cmd = SAVE_CALIB_CMD;
    if (i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1) != ESP_OK)
        return ESP_FAIL;

    return ESP_OK;
}

esp_err_t co2sensor_enable_aboc() {
    uint8_t cfg;
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    cfg = (cfg & ~BOC_CFG_MASK) | BOC_CFG_ABOC;
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    uint8_t cmd = SAVE_CONFIG_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

esp_err_t co2sensor_disable_aboc() {
    uint8_t cfg;
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    cfg = (cfg & ~BOC_CFG_MASK) | BOC_CFG_DISABLED;
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    uint8_t cmd = SAVE_CONFIG_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

esp_err_t co2sensor_soft_reset() {
    uint8_t cmd = SOFT_RESET_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

esp_err_t co2sensor_set_pressure(uint16_t pressure_hpa) {
    if (pressure_hpa < 750) pressure_hpa = 750;
    if (pressure_hpa > 1150) pressure_hpa = 1150;

    uint8_t high = (pressure_hpa >> 8) & 0xFF;
    uint8_t low  = pressure_hpa & 0xFF;

    if (i2c_write_bytes(CO2_ADDR, REG_PRESSURE_H, &high, 1) != ESP_OK ||
        i2c_write_bytes(CO2_ADDR, REG_PRESSURE_L, &low, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t co2sensor_disable_iir() {
    uint8_t cmd = DISABLE_IIR_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

esp_err_t co2sensor_enable_iir() {
    uint8_t cmd = ENABLE_IIR_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

esp_err_t co2sensor_wait_for_ready(int timeout_ms) {
    uint8_t prod_id;
    int elapsed = 0;

    while (elapsed < timeout_ms) {
        if (i2c_read_bytes(CO2_ADDR, REG_PROD_ID, &prod_id, 1) == ESP_OK) {
            ESP_LOGI(TAG_CO2, "Sensor ready, PROD_ID=0x%02X", prod_id);
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // try all 100 ms again
        elapsed += 100;
    }

    ESP_LOGE(TAG_CO2, "Sensor not ready after %d ms", timeout_ms);
    return ESP_ERR_TIMEOUT;
}
