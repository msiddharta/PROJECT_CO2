#ifndef BATTERY_H
#define BATTERY_H

#include "esp_err.h" // for esp_err_t

#define MAX17048_I2C_ADDR 0x36      // I2C address for MAX17048 battery sensor
#define REG_VCELL         0x02      // Register for battery voltage
#define REG_SOC           0x04      // Register for battery state of charge (SOC)

/**
 * @brief Get the battery voltage.
 *
 * @return float Battery voltage in volts.
 */
float battery_get_voltage(void);

/**
 * @brief Get the battery state of charge (SOC).
 *
 * @return float Battery SOC as a percentage.
 */
float battery_get_soc(void);

#endif // BATTERY_H
