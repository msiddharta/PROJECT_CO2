#ifndef BATTERY_H
#define BATTERY_H

#include "esp_err.h" // for esp_err_t

#define MAX17048_I2C_ADDR 0x36
#define REG_VCELL         0x02
#define REG_SOC           0x04

float battery_get_voltage(void);
float battery_get_soc(void);


#endif // BATTERY_H
