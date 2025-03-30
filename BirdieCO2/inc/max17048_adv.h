#ifndef MAX17048_ADV_H
#define MAX17048_ADV_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif



// I2C Address
#define MAX17048_I2C_ADDR 0x36

// Register Addresses
#define REG_VCELL         0x02
#define REG_SOC           0x04
#define REG_MODE          0x06
#define REG_VERSION       0x08
#define REG_HIBRT         0x0A
#define REG_CONFIG        0x0C
#define REG_VALERT_MIN    0x14
#define REG_VALERT_MAX    0x15
#define REG_CRATE         0x16
#define REG_VRESET        0x18
#define REG_STATUS        0x1A
#define REG_CMD           0xFE

// Alert Flag Bits
#define MAX1704X_ALERTFLAG_SOC_CHANGE       0x01
#define MAX1704X_ALERTFLAG_SOC_LOW          0x02
#define MAX1704X_ALERTFLAG_VOLTAGE_RESET    0x04
#define MAX1704X_ALERTFLAG_VOLTAGE_LOW      0x08
#define MAX1704X_ALERTFLAG_VOLTAGE_HIGH     0x10
#define MAX1704X_ALERTFLAG_RESET_INDICATOR  0x20
// (bits 6..7 are reserved)

// Call once at boot, after installing I2C driver. Optionally check device presence
esp_err_t max17048_init_check(void);

// Some advanced features:
esp_err_t max17048_quick_start(void);
esp_err_t max17048_set_alert_voltages(float minv, float maxv);
esp_err_t max17048_get_alert_voltages(float *minv, float *maxv);

esp_err_t max17048_set_reset_voltage(float reset_v);
float     max17048_get_reset_voltage(void);

esp_err_t max17048_set_activity_threshold(float volts);
float     max17048_get_activity_threshold(void);

esp_err_t max17048_set_hibernation_threshold(float per_hour);
float     max17048_get_hibernation_threshold(void);

bool      max17048_is_hibernating(void);

esp_err_t max17048_sleep_enable(bool enable);
esp_err_t max17048_sleep(bool enter);

esp_err_t max17048_clear_alert_flag(uint8_t flags);
uint8_t   max17048_get_alert_flags(void);

#ifdef __cplusplus
}
#endif

#endif // MAX17048_ADV_H
