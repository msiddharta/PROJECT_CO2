#include "max17048_adv.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MAX17048_ADV";

// ------------------------------------------------------------------------
// PUBLIC API Implementations
// ------------------------------------------------------------------------

esp_err_t max17048_init_check(void) {
    uint16_t version;
    esp_err_t err = i2c_read_u16_be(MAX17048_I2C_ADDR, REG_VERSION, &version);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read VERSION register (err=0x%x)", err);
        return err;
    }
    if ((version & 0xFFF0) != 0x0010) {
        ESP_LOGW(TAG, "MAX17048 not detected or not ready (version=0x%04X)", version);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MAX17048 version=0x%04X - device ready!", version);
    return ESP_OK;
}

esp_err_t max17048_quick_start(void) {
    uint16_t mode;
    esp_err_t err = i2c_read_u16_be(MAX17048_I2C_ADDR, REG_MODE, &mode);
    if (err != ESP_OK) return err;
    mode |= (1 << 6);
    return i2c_write_u16_be(MAX17048_I2C_ADDR, REG_MODE, mode);
}

esp_err_t max17048_set_alert_voltages(float minv, float maxv) {
    uint8_t min_val = (uint8_t)fminf(fmaxf(minv / 0.02f, 0), 255);
    uint8_t max_val = (uint8_t)fminf(fmaxf(maxv / 0.02f, 0), 255);

    esp_err_t err = i2c_write_bytes(MAX17048_I2C_ADDR, REG_VALERT_MIN, &min_val, 1);
    if (err != ESP_OK) return err;
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_VALERT_MAX, &max_val, 1);
}

esp_err_t max17048_get_alert_voltages(float *minv, float *maxv) {
    uint8_t min_raw, max_raw;
    esp_err_t err = i2c_read_bytes(MAX17048_I2C_ADDR, REG_VALERT_MIN, &min_raw, 1);
    if (err != ESP_OK) return err;
    err = i2c_read_bytes(MAX17048_I2C_ADDR, REG_VALERT_MAX, &max_raw, 1);
    if (err != ESP_OK) return err;

    if (minv) *minv = min_raw * 0.02f;
    if (maxv) *maxv = max_raw * 0.02f;
    return ESP_OK;
}

esp_err_t max17048_set_reset_voltage(float reset_v) {
    uint8_t reg_val;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_VRESET, &reg_val, 1) != ESP_OK)
        return ESP_FAIL;

    int steps = (int)((reset_v / 0.04f) + 0.5f);
    steps = fminf(fmaxf(steps, 0), 127);
    reg_val = (reg_val & 0x80) | (steps & 0x7F);

    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_VRESET, &reg_val, 1);
}

float max17048_get_reset_voltage(void) {
    uint8_t val;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_VRESET, &val, 1) != ESP_OK)
        return NAN;
    return (val & 0x7F) * 0.04f;
}

esp_err_t max17048_set_activity_threshold(float volts) {
    uint8_t steps = (uint8_t)fminf(fmaxf(volts / 0.00125f, 0), 255);
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_HIBRT + 1, &steps, 1);
}

float max17048_get_activity_threshold(void) {
    uint8_t val = 0;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_HIBRT + 1, &val, 1) != ESP_OK)
        return NAN;
    return val * 0.00125f;
}

esp_err_t max17048_set_hibernation_threshold(float per_hour) {
    uint8_t steps = (uint8_t)fminf(fmaxf(per_hour / 0.208f, 0), 255);
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_HIBRT, &steps, 1);
}

float max17048_get_hibernation_threshold(void) {
    uint8_t val = 0;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_HIBRT, &val, 1) != ESP_OK)
        return NAN;
    return val * 0.208f;
}

bool max17048_is_hibernating(void) {
    uint16_t mode;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_MODE, &mode) != ESP_OK)
        return false;
    return (mode & (1 << 4)) != 0;
}

esp_err_t max17048_sleep_enable(bool enable) {
    uint16_t mode;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_MODE, &mode) != ESP_OK)
        return ESP_FAIL;

    if (enable) mode |= (1 << 5);
    else        mode &= ~(1 << 5);

    return i2c_write_u16_be(MAX17048_I2C_ADDR, REG_MODE, mode);
}

esp_err_t max17048_sleep(bool enter) {
    uint16_t config;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_CONFIG, &config) != ESP_OK)
        return ESP_FAIL;

    if (enter) config |= (1 << 15);
    else       config &= ~(1 << 15);

    return i2c_write_u16_be(MAX17048_I2C_ADDR, REG_CONFIG, config);
}

esp_err_t max17048_clear_alert_flag(uint8_t flags) {
    uint8_t status;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_STATUS, &status, 1) != ESP_OK)
        return ESP_FAIL;

    status = (status & ~flags) & 0x7F;
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_STATUS, &status, 1);
}

uint8_t max17048_get_alert_flags(void) {
    uint8_t status;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_STATUS, &status, 1) == ESP_OK)
        return status & 0x7F;
    return 0;
}
