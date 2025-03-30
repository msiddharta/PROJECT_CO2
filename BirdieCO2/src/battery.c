#include "battery.h"
#include "esp_log.h"
#include "i2c_bus.h"
#include "max17048_adv.h"

#define TAG_BATT "BATTERY"

// Returns battery voltage in volts from MAX17048
float battery_get_voltage(void) {
    uint16_t raw;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_VCELL, &raw) != ESP_OK) {
        ESP_LOGE(TAG_BATT, "Failed to read VCELL register");
        return -1.0f;
    }
    return raw * 78.125e-6f;  // LSB = 78.125 µV
}

// Returns battery state-of-charge in percent (0-100%)
float battery_get_soc(void) {
    uint16_t raw;
    if (i2c_read_u16_be(MAX17048_I2C_ADDR, REG_SOC, &raw) != ESP_OK) {
        ESP_LOGE(TAG_BATT, "Failed to read SOC register");
        return -1.0f;
    }
    return raw / 256.0f;  // LSB = 1/256 %
}
