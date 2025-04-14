#include "co2sensor.h"          // Header for CO2 sensor definitions and function prototypes.
#include "esp_log.h"            // ESP logging functions.
#include "i2c_bus.h"            // I2C bus utility functions.
#include "freertos/FreeRTOS.h"  // FreeRTOS base header.
#include "freertos/task.h"      // FreeRTOS task delay functions.

// Tag used for logging messages for the CO2 sensor module.
static const char *TAG_CO2 = "CO2_SENSOR";

/**
 * @brief Trigger a single-shot CO2 measurement.
 *
 * Reads the measurement configuration register, sets the sensor to single-shot mode,
 * and then waits for the data-ready flag (DRDY_BIT) in the measurement status register.
 * If data is ready within the defined number of retries, the function returns ESP_OK;
 * otherwise, it returns ESP_ERR_TIMEOUT.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_trigger_measurement() {
    uint8_t cfg;
    // Read the current measurement configuration from the sensor.
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    // Clear the operation mode bits (bits 1:0) in the configuration.
    cfg &= ~0x03;
    // Set to single-shot measurement mode by updating OP_MODE to 0b01.
    cfg |= 0x01;
    // Write the updated configuration back to the sensor.
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    // Poll the measurement status register until the data-ready flag (DRDY_BIT) is set.
    for (int i = 0; i < MAX_READY_RETRIES; i++) {
        uint8_t status = 0;
        // Check if the sensor is reporting data ready.
        if (i2c_read_bytes(CO2_ADDR, REG_MEAS_STS, &status, 1) == ESP_OK &&
            (status & DRDY_BIT)) {
            return ESP_OK;
        }
        // Delay before retrying to allow the sensor time to complete the measurement.
        vTaskDelay(pdMS_TO_TICKS(MEAS_DELAY_MS));
    }
    // Log a warning if the sensor did not indicate data ready after the timeout.
    ESP_LOGW(TAG_CO2, "Timeout waiting for data ready");
    return ESP_ERR_TIMEOUT;
}

/**
 * @brief Read CO2 concentration in parts per million (ppm) from the sensor.
 *
 * Reads the high and low bytes from the respective registers and combines them
 * into a single 16-bit value, which is stored in the variable pointed to by ppm_out.
 *
 * @param ppm_out Pointer to the variable where the CO2 concentration will be stored.
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_read_ppm(uint16_t *ppm_out) {
    // Ensure the output pointer is valid.
    if (!ppm_out) return ESP_ERR_INVALID_ARG;
    
    uint8_t high = 0, low = 0;
    // Read the high byte of the CO2 concentration.
    if (i2c_read_bytes(CO2_ADDR, REG_CO2PPM_H, &high, 1) != ESP_OK ||
        // Read the low byte of the CO2 concentration.
        i2c_read_bytes(CO2_ADDR, REG_CO2PPM_L, &low, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    // Combine the high and low bytes to form a 16-bit value (big-endian format).
    *ppm_out = ((uint16_t)high << 8) | low;
    return ESP_OK;
}

/**
 * @brief Calibrate the CO2 sensor with a given reference ppm value.
 *
 * Constrains the reference value to between 350 and 1500 ppm, writes the calibration
 * reference high and low bytes to their respective registers, and then issues a
 * save calibration command.
 *
 * @param ref_ppm The reference CO2 concentration (ppm) for calibration.
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_calibrate(uint16_t ref_ppm) {
    // Constrain the input calibration value within an acceptable range.
    if (ref_ppm < 350) ref_ppm = 350;
    if (ref_ppm > 1500) ref_ppm = 1500;

    // Split the 16-bit calibration value into high and low bytes.
    uint8_t high = (ref_ppm >> 8) & 0xFF;
    uint8_t low  = ref_ppm & 0xFF;
    // Write the high byte to the calibration reference high register.
    if (i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_H, &high, 1) != ESP_OK ||
        // Write the low byte to the calibration reference low register.
        i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_L, &low, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    // Issue the save calibration command to store the calibration data permanently.
    uint8_t cmd = SAVE_CALIB_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

/**
 * @brief Force compensation adjustment on the sensor.
 *
 * Performs forced compensation by updating the configuration register to set
 * forced compensation mode (BOC_CFG_FORCED). It then triggers three measurements,
 * each followed by a delay, and finally saves the forced compensation settings
 * to non-volatile memory.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_force_compensation() {
    uint8_t cfg;
    ESP_LOGW("CALIB", "Forced compensation started!");
    
    // Read the current measurement configuration.
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    // Set the BOC configuration to forced compensation mode.
    cfg = (cfg & ~BOC_CFG_MASK) | BOC_CFG_FORCED;
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK)
        return ESP_FAIL;

    // Delay briefly to allow the configuration to take effect.
    vTaskDelay(pdMS_TO_TICKS(200));

    // Perform 3 measurement cycles to apply forced compensation.
    for (int i = 0; i < 3; i++) {
        if (co2sensor_trigger_measurement() != ESP_OK)
            return ESP_FAIL;
        // Wait between measurements.
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGW("CALIB", "Forced compensation measurement %d", i + 1);
    }

    // Save the forced compensation settings in non-volatile memory.
    uint8_t cmd = SAVE_CALIB_CMD;
    if (i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1) != ESP_OK)
        return ESP_FAIL;

    return ESP_OK;
}

/**
 * @brief Enable Auto Baseline Offset Compensation (ABOC) on the sensor.
 *
 * Updates the measurement configuration register to set the ABOC mode, then
 * saves the configuration to the sensor.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_enable_aboc() {
    uint8_t cfg;
    // Read current configuration.
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    // Update BOC configuration bits to enable ABOC.
    cfg = (cfg & ~BOC_CFG_MASK) | BOC_CFG_ABOC;
    // Write the updated configuration.
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    // Save the configuration settings.
    uint8_t cmd = SAVE_CONFIG_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

/**
 * @brief Disable Auto Baseline Offset Compensation (ABOC) on the sensor.
 *
 * Clears the ABOC mode in the measurement configuration and saves the updated
 * configuration to the sensor.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_disable_aboc() {
    uint8_t cfg;
    // Read current measurement configuration.
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    // Clear the BOC configuration bits and set them to disabled.
    cfg = (cfg & ~BOC_CFG_MASK) | BOC_CFG_DISABLED;
    // Write the new configuration back to the sensor.
    if (i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &cfg, 1) != ESP_OK) return ESP_FAIL;
    // Save the disabled configuration.
    uint8_t cmd = SAVE_CONFIG_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

/**
 * @brief Perform a soft reset of the CO2 sensor.
 *
 * Sends the soft reset command to the sensor to reinitialize its internal state.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_soft_reset() {
    uint8_t cmd = SOFT_RESET_CMD;  // Command value for soft reset.
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

/**
 * @brief Set the ambient pressure value in the sensor.
 *
 * The sensor uses ambient pressure data (in hPa) to adjust readings. This function
 * limits the pressure to a valid range, splits the value into high and low bytes, and
 * writes them to the respective pressure registers.
 *
 * @param pressure_hpa Ambient pressure in hectoPascals.
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_set_pressure(uint16_t pressure_hpa) {
    // Constrain the pressure value to within acceptable limits.
    if (pressure_hpa < 750) pressure_hpa = 750;
    if (pressure_hpa > 1150) pressure_hpa = 1150;

    // Split the 16-bit pressure value into high and low bytes.
    uint8_t high = (pressure_hpa >> 8) & 0xFF;
    uint8_t low  = pressure_hpa & 0xFF;

    // Write the high byte to the pressure high register.
    if (i2c_write_bytes(CO2_ADDR, REG_PRESSURE_H, &high, 1) != ESP_OK ||
        // Write the low byte to the pressure low register.
        i2c_write_bytes(CO2_ADDR, REG_PRESSURE_L, &low, 1) != ESP_OK) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

/**
 * @brief Disable the IIR filter on the sensor.
 *
 * Sends a command to disable the sensor's IIR filter, which may affect how raw data
 * is processed.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_disable_iir() {
    uint8_t cmd = DISABLE_IIR_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

/**
 * @brief Enable the IIR filter on the sensor.
 *
 * Sends a command to enable the sensor's IIR filter for smoothing the readings.
 *
 * @return esp_err_t ESP_OK on success or an appropriate error code on failure.
 */
esp_err_t co2sensor_enable_iir() {
    uint8_t cmd = ENABLE_IIR_CMD;
    return i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &cmd, 1);
}

/**
 * @brief Wait for the sensor to become ready.
 *
 * Polls the sensor's product ID register until it responds as ready or until the
 * specified timeout is reached. Logs the sensor’s product ID when found.
 *
 * @param timeout_ms The maximum time in milliseconds to wait.
 * @return esp_err_t ESP_OK if the sensor is ready or ESP_ERR_TIMEOUT otherwise.
 */
esp_err_t co2sensor_wait_for_ready(int timeout_ms) {
    uint8_t prod_id;
    int elapsed = 0;

    // Continue polling until timeout is reached.
    while (elapsed < timeout_ms) {
        // Attempt to read the product ID.
        if (i2c_read_bytes(CO2_ADDR, REG_PROD_ID, &prod_id, 1) == ESP_OK) {
            ESP_LOGI(TAG_CO2, "Sensor ready, PROD_ID=0x%02X", prod_id);
            return ESP_OK;
        }
        // Delay for 100 ms before the next polling attempt.
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
    }

    // Log an error if the sensor isn't ready within the timeout period.
    ESP_LOGE(TAG_CO2, "Sensor not ready after %d ms", timeout_ms);
    return ESP_ERR_TIMEOUT;
}
