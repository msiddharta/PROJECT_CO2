#include "max17048_adv.h"   // Advanced MAX17048 definitions
#include "i2c_bus.h"         // I2C communication utilities
#include "esp_log.h"         // Logging functions
#include <math.h>            // Math functions for fminf, fmaxf, etc.

static const char *TAG = "MAX17048_ADV"; // Logging tag for MAX17048 module

// ------------------------------------------------------------------------
// PUBLIC API Implementations
// ------------------------------------------------------------------------

/**
 * @brief Initialize and check the MAX17048 sensor.
 *
 * Reads the VERSION register to verify that the sensor is present and ready.
 * Logs an error if the register read fails or if the version does not match the expected value.
 *
 * @return esp_err_t ESP_OK if the sensor is ready; otherwise, an error code.
 */
esp_err_t max17048_init_check(void) {
    uint16_t version;
    // Read 16-bit version from the sensor's VERSION register.
    esp_err_t err = i2c_read_u16_be(MAX17048_I2C_ADDR, REG_VERSION, &version);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read VERSION register (err=0x%x)", err);
        return err;
    }
    // Verify that the version matches the expected pattern (upper 12 bits should be 0x0010).
    if ((version & 0xFFF0) != 0x0010) {
        ESP_LOGW(TAG, "MAX17048 not detected or not ready (version=0x%04X)", version);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "MAX17048 version=0x%04X - device ready!", version);
    return ESP_OK;
}

/**
 * @brief Perform a quick start on the sensor.
 *
 * Reads the MODE register, sets the quick start bit (bit 6), and writes the result back.
 *
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_quick_start(void) {
    uint16_t mode;
    // Read the current mode settings.
    esp_err_t err = i2c_read_u16_be(MAX17048_I2C_ADDR, REG_MODE, &mode);
    if (err != ESP_OK) return err;
    // Set bit 6 to perform a quick start.
    mode |= (1 << 6);
    // Write updated mode back to the sensor.
    return i2c_write_u16_be(MAX17048_I2C_ADDR, REG_MODE, mode);
}

/**
 * @brief Set the sensor's alert voltage thresholds.
 *
 * Converts the provided minimum and maximum voltage (in volts) to sensor units
 * by dividing by 0.02 and clamping to [0,255], then writes these values to the
 * appropriate registers.
 *
 * @param minv Minimum alert voltage in volts.
 * @param maxv Maximum alert voltage in volts.
 * @return esp_err_t Result of the I2C write operations.
 */
esp_err_t max17048_set_alert_voltages(float minv, float maxv) {
    // Convert voltage values to sensor steps (0.02 V/step).
    uint8_t min_val = (uint8_t)fminf(fmaxf(minv / 0.02f, 0), 255);
    uint8_t max_val = (uint8_t)fminf(fmaxf(maxv / 0.02f, 0), 255);

    // Write minimum threshold.
    esp_err_t err = i2c_write_bytes(MAX17048_I2C_ADDR, REG_VALERT_MIN, &min_val, 1);
    if (err != ESP_OK) return err;
    // Write maximum threshold.
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_VALERT_MAX, &max_val, 1);
}

/**
 * @brief Get the current alert voltage thresholds.
 *
 * Reads the raw alert voltage values from the sensor registers and converts them
 * to volts by multiplying by 0.02.
 *
 * @param minv Pointer to store the minimum voltage (in volts).
 * @param maxv Pointer to store the maximum voltage (in volts).
 * @return esp_err_t ESP_OK if successful; an error code otherwise.
 */
esp_err_t max17048_get_alert_voltages(float *minv, float *maxv) {
    uint8_t min_raw, max_raw;
    // Read minimum alert voltage.
    esp_err_t err = i2c_read_bytes(MAX17048_I2C_ADDR, REG_VALERT_MIN, &min_raw, 1);
    if (err != ESP_OK) return err;
    // Read maximum alert voltage.
    err = i2c_read_bytes(MAX17048_I2C_ADDR, REG_VALERT_MAX, &max_raw, 1);
    if (err != ESP_OK) return err;

    // Convert raw values to volts.
    if (minv) *minv = min_raw * 0.02f;
    if (maxv) *maxv = max_raw * 0.02f;
    return ESP_OK;
}

/**
 * @brief Set the sensor's reset voltage.
 *
 * Reads the current VRESET register value, calculates the required steps for
 * the new reset voltage (using 0.04 V/step), clamps the value to 0-127, and updates
 * the register while preserving the MSB.
 *
 * @param reset_v Desired reset voltage in volts.
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_set_reset_voltage(float reset_v) {
    uint8_t reg_val;
    // Read current reset voltage register.
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_VRESET, &reg_val, 1) != ESP_OK)
        return ESP_FAIL;

    // Calculate the number of steps (each step = 0.04 V), rounding to the nearest integer.
    int steps = (int)((reset_v / 0.04f) + 0.5f);
    // Clamp steps between 0 and 127.
    steps = fminf(fmaxf(steps, 0), 127);
    // Preserve MSB (bit 7) and update the lower 7 bits with steps.
    reg_val = (reg_val & 0x80) | (steps & 0x7F);

    // Write the new reset voltage value to the sensor.
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_VRESET, &reg_val, 1);
}

/**
 * @brief Retrieve the sensor's reset voltage.
 *
 * Reads the VRESET register, extracts the lower 7 bits, and converts them to a voltage
 * in volts using a 0.04 V per step conversion factor.
 *
 * @return float Reset voltage in volts, or NaN if the read fails.
 */
float max17048_get_reset_voltage(void) {
    uint8_t val;
    // Read reset voltage register.
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_VRESET, &val, 1) != ESP_OK)
        return NAN;
    // Convert the lower 7 bits to voltage.
    return (val & 0x7F) * 0.04f;
}

/**
 * @brief Set the activity threshold voltage.
 *
 * Converts the provided voltage (in volts) into sensor steps (using 0.00125 V/step)
 * and writes the value to the register at REG_HIBRT + 1.
 *
 * @param volts Voltage threshold in volts.
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_set_activity_threshold(float volts) {
    uint8_t steps = (uint8_t)fminf(fmaxf(volts / 0.00125f, 0), 255);
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_HIBRT + 1, &steps, 1);
}

/**
 * @brief Get the activity threshold voltage.
 *
 * Reads the register at REG_HIBRT + 1 and converts the raw value to volts
 * using a 0.00125 V per step conversion factor.
 *
 * @return float Voltage threshold in volts, or NaN if the read fails.
 */
float max17048_get_activity_threshold(void) {
    uint8_t val = 0;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_HIBRT + 1, &val, 1) != ESP_OK)
        return NAN;
    return val * 0.00125f;
}

/**
 * @brief Set the hibernation threshold.
 *
 * Converts the given hibernation threshold (in per hour units) into sensor steps,
 * using a 0.208 per step conversion factor, and writes the value to the REG_HIBRT register.
 *
 * @param per_hour Hibernation threshold in per hour units.
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_set_hibernation_threshold(float per_hour) {
    uint8_t steps = (uint8_t)fminf(fmaxf(per_hour / 0.208f, 0), 255);
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_HIBRT, &steps, 1);
}

/**
 * @brief Get the hibernation threshold.
 *
 * Reads the REG_HIBRT register and converts the raw value to hibernation threshold
 * in per hour units using a 0.208 conversion factor.
 *
 * @return float Hibernation threshold, or NaN if the read fails.
 */
float max17048_get_hibernation_threshold(void) {
    uint8_t val = 0;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_HIBRT, &val, 1) != ESP_OK)
        return NAN;
    return val * 0.208f;
}

/**
 * @brief Check whether the sensor is in hibernation mode.
 *
 * Reads the MODE register and checks the hibernation bit (bit 4).
 *
 * @return bool true if the sensor is hibernating; false otherwise.
 */
bool max17048_is_hibernating(void) {
    uint16_t mode;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_MODE, &mode) != ESP_OK)
        return false;
    return (mode & (1 << 4)) != 0;
}

/**
 * @brief Enable or disable sleep mode.
 *
 * Reads the MODE register, sets or clears the sleep enable bit (bit 5) based on the input,
 * and writes the updated mode back to the sensor.
 *
 * @param enable true to enable sleep mode; false to disable.
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_sleep_enable(bool enable) {
    uint16_t mode;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_MODE, &mode) != ESP_OK)
        return ESP_FAIL;

    if (enable)
        mode |= (1 << 5);   // Set sleep enable bit
    else
        mode &= ~(1 << 5);  // Clear sleep enable bit

    return i2c_write_u16_be(MAX17048_I2C_ADDR, REG_MODE, mode);
}

/**
 * @brief Enter or exit sleep mode.
 *
 * Reads the CONFIG register, sets or clears the sleep bit (bit 15) based on the input,
 * and writes the updated configuration back to the sensor.
 *
 * @param enter true to put the sensor to sleep; false to wake it up.
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_sleep(bool enter) {
    uint16_t config;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_CONFIG, &config) != ESP_OK)
        return ESP_FAIL;

    if (enter)
        config |= (1 << 15);  // Set sleep bit
    else
        config &= ~(1 << 15); // Clear sleep bit

    return i2c_write_u16_be(MAX17048_I2C_ADDR, REG_CONFIG, config);
}

/**
 * @brief Clear specific alert flags.
 *
 * Reads the STATUS register, clears the bits specified in the flags parameter (only lower 7 bits),
 * and writes the updated status back to the sensor.
 *
 * @param flags Bitmask of alert flags to clear.
 * @return esp_err_t Result of the I2C write operation.
 */
esp_err_t max17048_clear_alert_flag(uint8_t flags) {
    uint8_t status;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_STATUS, &status, 1) != ESP_OK)
        return ESP_FAIL;

    // Clear the specified flags and ensure only lower 7 bits remain.
    status = (status & ~flags) & 0x7F;
    return i2c_write_bytes(MAX17048_I2C_ADDR, REG_STATUS, &status, 1);
}

/**
 * @brief Get current alert flags.
 *
 * Reads the STATUS register and returns the lower 7 bits which contain the alert flags.
 *
 * @return uint8_t The current alert flags; returns 0 if the read fails.
 */
uint8_t max17048_get_alert_flags(void) {
    uint8_t status;
    if (i2c_read_bytes(MAX17048_I2C_ADDR, REG_STATUS, &status, 1) == ESP_OK)
        return status & 0x7F;
    return 0;
}
