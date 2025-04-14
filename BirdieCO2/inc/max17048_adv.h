#ifndef MAX17048_ADV_H
#define MAX17048_ADV_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// I2C Address for MAX17048
#define MAX17048_I2C_ADDR 0x36

// Register Addresses
#define REG_VCELL         0x02  // Battery voltage register
#define REG_SOC           0x04  // Battery SOC register
#define REG_MODE          0x06  // Mode register
#define REG_VERSION       0x08  // Version register
#define REG_HIBRT         0x0A  // Hibernation register
#define REG_CONFIG        0x0C  // Configuration register
#define REG_VALERT_MIN    0x14  // Minimum alert voltage register
#define REG_VALERT_MAX    0x15  // Maximum alert voltage register
#define REG_CRATE         0x16  // Charge rate register
#define REG_VRESET        0x18  // Voltage reset register
#define REG_STATUS        0x1A  // Status register
#define REG_CMD           0xFE  // Command register

// Alert Flag Bits
#define MAX1704X_ALERTFLAG_SOC_CHANGE       0x01  // SOC change alert
#define MAX1704X_ALERTFLAG_SOC_LOW            0x02  // Low SOC alert
#define MAX1704X_ALERTFLAG_VOLTAGE_RESET      0x04  // Voltage reset alert
#define MAX1704X_ALERTFLAG_VOLTAGE_LOW        0x08  // Low voltage alert
#define MAX1704X_ALERTFLAG_VOLTAGE_HIGH       0x10  // High voltage alert
#define MAX1704X_ALERTFLAG_RESET_INDICATOR    0x20  // Reset indicator alert
// (bits 6..7 are reserved)

// Initialize sensor and check for device presence (call once at boot)
esp_err_t max17048_init_check(void);

// Perform a quick start
esp_err_t max17048_quick_start(void);

// Set alert voltage thresholds
esp_err_t max17048_set_alert_voltages(float minv, float maxv);

// Get current alert voltage thresholds
esp_err_t max17048_get_alert_voltages(float *minv, float *maxv);

// Set reset voltage value
esp_err_t max17048_set_reset_voltage(float reset_v);

// Get current reset voltage
float     max17048_get_reset_voltage(void);

// Set activity threshold voltage
esp_err_t max17048_set_activity_threshold(float volts);

// Get current activity threshold voltage
float     max17048_get_activity_threshold(void);

// Set hibernation threshold (per hour)
esp_err_t max17048_set_hibernation_threshold(float per_hour);

// Get current hibernation threshold
float     max17048_get_hibernation_threshold(void);

// Check if sensor is in hibernation mode
bool      max17048_is_hibernating(void);

// Enable or disable sleep mode
esp_err_t max17048_sleep_enable(bool enable);

// Put sensor to sleep or wake it up
esp_err_t max17048_sleep(bool enter);

// Clear specific alert flags
esp_err_t max17048_clear_alert_flag(uint8_t flags);

// Retrieve current alert flags
uint8_t   max17048_get_alert_flags(void);

#ifdef __cplusplus
}
#endif

#endif // MAX17048_ADV_H
