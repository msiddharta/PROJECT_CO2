#include "battery.h"        // Include battery header for function prototypes.
#include "esp_log.h"        // Include logging functions.
#include "i2c_bus.h"        // Include I2C utility functions for communication.
#include "max17048_adv.h"   // Include advanced definitions for MAX17048 sensor.

#define TAG_BATT "BATTERY"  // Logging tag for battery-related messages

/**
 * @brief Get the battery voltage from the MAX17048 sensor.
 *
 * This function reads a 16-bit big-endian value from the VCELL register
 * and converts it to a voltage in volts. The conversion factor is based on the
 * sensor's resolution (LSB = 78.125 µV).
 *
 * @return float Battery voltage in volts; returns -1.0f on failure.
 */
float battery_get_voltage(void) {
    uint16_t raw;  // Variable to store the raw 16-bit reading from VCELL register.

    // Read the VCELL register from the sensor using I2C.
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_VCELL, &raw) != ESP_OK) {
        // Log an error if reading fails.
        ESP_LOGE(TAG_BATT, "Failed to read VCELL register");
        return -1.0f;
    }
    
    // Convert the raw value to voltage.
    // Each LSB represents 78.125 µV, so multiply raw by 78.125e-6.
    return raw * 78.125e-6f;
}

/**
 * @brief Get the battery state-of-charge (SOC) from the MAX17048 sensor.
 *
 * This function reads a 16-bit big-endian value from the SOC register and
 * converts it to a percentage. The conversion factor is based on the sensor's
 * resolution (LSB = 1/256 %).
 *
 * @return float Battery state-of-charge in percent; returns -1.0f on failure.
 */
float battery_get_soc(void) {
    uint16_t raw;  // Variable to store the raw 16-bit reading from SOC register.

    // Read the SOC register from the sensor using I2C.
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_SOC, &raw) != ESP_OK) {
        // Log an error if reading fails.
        ESP_LOGE(TAG_BATT, "Failed to read SOC register");
        return -1.0f;
    }
    
    // Convert the raw value to percent.
    // Each LSB represents 1/256 %, so divide raw by 256.
    return raw / 256.0f;
}
