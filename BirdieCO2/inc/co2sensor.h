#ifndef CO2SENSOR_H
#define CO2SENSOR_H

#include "esp_system.h"

// PASCO2V15 I2C Address
#define CO2_ADDR     0x28

// Register Definitions
#define REG_MEAS_CFG        0x04  // Measurement Configuration Register
#define REG_MEAS_STS        0x07  // Measurement Status Register (DRDY bit)
#define REG_CO2PPM_H        0x05  // High byte of CO2 concentration
#define REG_CO2PPM_L        0x06  // Low byte of CO2 concentration
#define REG_CALIB_REF_H     0x0D  // Calibration reference high byte
#define REG_CALIB_REF_L     0x0E  // Calibration reference low byte
#define REG_SENS_RST        0x10  // Reset register (to apply calibration)

// Function Declarations
void co2sensor_init(int value);
void co2sensor_calibrate(uint16_t ref_co2_ppm);
void co2sensor_enable_aboc();
bool co2sensor_is_data_ready();
void wait_for_data_ready();
uint16_t co2sensor_read_co2();
void trigger_single_measurement();
void calibrate_sensor(uint16_t ref_co2_ppm);
void perform_forced_compensation(void);
void enable_aboc();
bool is_data_ready();

#endif // CO2SENSOR_H
