#ifndef CO2SENSOR_H
#define CO2SENSOR_H

#include "esp_system.h"
#include "driver/i2c.h"

// I2C Configuration
#define I2C_MASTER_SCL_IO   17   // SCL GPIO (Change if needed)
#define I2C_MASTER_SDA_IO   18   // SDA GPIO (Change if needed)
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_TIMEOUT_MS      2000

// PASCO2V15 I2C Address
#define PASCO2_I2C_ADDR     0x28

// Register Definitions
#define REG_MEAS_CFG        0x04  // Measurement Configuration Register
#define REG_MEAS_STS        0x07  // Measurement Status Register (DRDY bit)
#define REG_CO2PPM_H        0x05  // High byte of CO2 concentration
#define REG_CO2PPM_L        0x06  // Low byte of CO2 concentration
#define REG_CALIB_REF_H     0x0D  // Calibration reference high byte
#define REG_CALIB_REF_L     0x0E  // Calibration reference low byte
#define REG_SENS_RST        0x10  // Reset register (to apply calibration)
#define REG_MEAS_CFG        0x04  // Measurement configuration register

// Function Declarations
void co2sensor_init();
void co2sensor_calibrate(uint16_t ref_co2_ppm);
void co2sensor_enable_aboc();
bool co2sensor_is_data_ready();
uint16_t co2sensor_read_co2();
void i2c_scan();
void trigger_single_measurement();
void calibrate_sensor(uint16_t ref_co2_ppm);
void enable_aboc();
bool is_data_ready();

#endif // CO2SENSOR_H
