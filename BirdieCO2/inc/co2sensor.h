#ifndef CO2SENSOR_H
#define CO2SENSOR_H

#include "esp_system.h"

// I2C address of the PASCO2V15 sensor
#define CO2_ADDR 0x28

// Register addresses
#define REG_PROD_ID        0x00
#define REG_SENS_STS       0x01
#define REG_MEAS_RATE_H    0x02
#define REG_MEAS_RATE_L    0x03
#define REG_MEAS_CFG       0x04
#define REG_CO2PPM_H       0x05
#define REG_CO2PPM_L       0x06
#define REG_MEAS_STS       0x07
#define REG_INT_CFG        0x08
#define REG_ALARM_TH_H     0x09
#define REG_ALARM_TH_L     0x0A
#define REG_PRESSURE_H     0x0B
#define REG_PRESSURE_L     0x0C
#define REG_CALIB_REF_H    0x0D
#define REG_CALIB_REF_L    0x0E
#define REG_SCRATCH_PAD    0x0F
#define REG_SENS_RST       0x10

// Bit definitions
#define DRDY_BIT           (1 << 4)
#define BOC_CFG_MASK       (0x03 << 2)  // Bits 3:2
#define BOC_CFG_DISABLED   (0x00 << 2)
#define BOC_CFG_ABOC       (0x01 << 2)
#define BOC_CFG_FORCED     (0x02 << 2)

// Command codes
#define SOFT_RESET_CMD     0xA3
#define RESET_ABOC_CMD     0xBC
#define DISABLE_VDD_COMP   0xCD
#define SAVE_CALIB_CMD     0xCF
#define DISABLE_IIR_CMD    0xDF
#define RESET_FORCE_COMP   0xFC
#define ENABLE_IIR_CMD     0xFE
#define SAVE_CONFIG_CMD    0xC3

// Constants
#define MAX_READY_RETRIES 10
#define MEAS_DELAY_MS     500

// Function declarations
esp_err_t co2sensor_soft_reset();
esp_err_t co2sensor_set_pressure(uint16_t pressure_hpa);
esp_err_t co2sensor_trigger_measurement();
esp_err_t co2sensor_read_ppm(uint16_t *ppm_out);
esp_err_t co2sensor_calibrate(uint16_t ref_ppm);
esp_err_t co2sensor_force_compensation();
esp_err_t co2sensor_enable_aboc();
esp_err_t co2sensor_disable_aboc();
esp_err_t co2sensor_disable_iir();
esp_err_t co2sensor_enable_iir();
esp_err_t co2sensor_wait_for_ready(int timeout_ms);

#endif // CO2SENSOR_H
