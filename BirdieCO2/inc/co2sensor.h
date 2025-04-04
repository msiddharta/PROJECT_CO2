#ifndef CO2SENSOR_H
#define CO2SENSOR_H

#include "esp_system.h"

// PASCO2V15 I2C Address
#define CO2_ADDR     0x28

// Register Definitions
#define REG_MEAS_CFG        0x04
#define REG_MEAS_STS        0x07
#define REG_CO2PPM_H        0x05
#define REG_CO2PPM_L        0x06
#define REG_CALIB_REF_H     0x0D
#define REG_CALIB_REF_L     0x0E
#define REG_SENS_RST        0x10

#define DRDY_BIT          (1 << 4)
#define MAX_READY_RETRIES 10
#define MEAS_DELAY_MS     500
#define SAVE_CALIB_CMD    0xCF
#define SOFT_RESET_CMD    0xA3
#define ABOC_ENABLE_CMD   0x01

// Function Declarations
esp_err_t co2sensor_calibrate(uint16_t ref_co2_ppm);
esp_err_t co2sensor_enable_aboc();
esp_err_t co2sensor_soft_reset();
esp_err_t trigger_single_measurement();
esp_err_t perform_forced_compensation();
esp_err_t co2sensor_read_co2(uint16_t *ppm_out);

#endif // CO2SENSOR_H
