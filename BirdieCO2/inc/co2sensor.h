#ifndef CO2SENSOR_H
#define CO2SENSOR_H

#include "esp_system.h"

/**
 * @file co2sensor.h
 * @brief Header file for PASCO2V15 CO2 sensor interface over I2C.
 *
 * This file defines register addresses, bit definitions, command codes,
 * and function prototypes for operating the PASCO2V15 CO2 sensor.
 */

// I2C address of the PASCO2V15 sensor.
#define CO2_ADDR 0x28

// Register addresses
// These macros define the addresses of various registers used by the CO2 sensor.
#define REG_PROD_ID        0x00  // Product ID register.
#define REG_SENS_STS       0x01  // Sensor status register.
#define REG_MEAS_RATE_H    0x02  // High byte of measurement rate.
#define REG_MEAS_RATE_L    0x03  // Low byte of measurement rate.
#define REG_MEAS_CFG       0x04  // Measurement configuration register.
#define REG_CO2PPM_H       0x05  // High byte of CO2 concentration in ppm.
#define REG_CO2PPM_L       0x06  // Low byte of CO2 concentration in ppm.
#define REG_MEAS_STS       0x07  // Measurement status register.
#define REG_INT_CFG        0x08  // Interrupt configuration register.
#define REG_ALARM_TH_H     0x09  // High byte of alarm threshold.
#define REG_ALARM_TH_L     0x0A  // Low byte of alarm threshold.
#define REG_PRESSURE_H     0x0B  // High byte of pressure reading.
#define REG_PRESSURE_L     0x0C  // Low byte of pressure reading.
#define REG_CALIB_REF_H    0x0D  // High byte of calibration reference.
#define REG_CALIB_REF_L    0x0E  // Low byte of calibration reference.
#define REG_SCRATCH_PAD    0x0F  // Scratch pad register for temporary data.
#define REG_SENS_RST       0x10  // Sensor reset register.

// Bit definitions
// These definitions are used to manipulate specific bits in sensor registers.
#define DRDY_BIT           (1 << 4)         // Data Ready bit mask.
#define BOC_CFG_MASK       (0x03 << 2)      // Mask for BOC configuration bits (bits 3:2).
#define BOC_CFG_DISABLED   (0x00 << 2)      // BOC configuration: disabled.
#define BOC_CFG_ABOC       (0x01 << 2)      // BOC configuration: auto baseline offset compensation.
#define BOC_CFG_FORCED     (0x02 << 2)      // BOC configuration: forced compensation.

// Command codes
// Commands for controlling various sensor functions.
#define SOFT_RESET_CMD     0xA3             // Command to perform a soft reset.
#define RESET_ABOC_CMD     0xBC             // Command to reset auto baseline offset compensation.
#define DISABLE_VDD_COMP   0xCD             // Command to disable voltage compensation.
#define SAVE_CALIB_CMD     0xCF             // Command to save calibration data.
#define DISABLE_IIR_CMD    0xDF             // Command to disable IIR filtering.
#define RESET_FORCE_COMP   0xFC             // Command to reset forced compensation.
#define ENABLE_IIR_CMD     0xFE             // Command to enable IIR filtering.
#define SAVE_CONFIG_CMD    0xC3             // Command to save sensor configuration.

// Constants
// Various constants used for sensor timing and retries.
#define MAX_READY_RETRIES 10                // Maximum number of retries waiting for sensor readiness.
#define MEAS_DELAY_MS     500               // Delay in milliseconds for measurement operations.

// Function declarations

/**
 * @brief Perform a soft reset of the CO2 sensor.
 *
 * This function issues a soft reset command to reinitialize the sensor.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_soft_reset();

/**
 * @brief Set the ambient pressure value on the CO2 sensor.
 *
 * This function configures the sensor with the ambient pressure in hectoPascals (hPa),
 * which may influence sensor readings.
 *
 * @param pressure_hpa Pressure value in hPa.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_set_pressure(uint16_t pressure_hpa);

/**
 * @brief Trigger a new CO2 measurement.
 *
 * This function starts a measurement operation on the sensor.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_trigger_measurement();

/**
 * @brief Read the CO2 concentration from the sensor.
 *
 * This function retrieves the measured CO2 concentration (in parts per million) from the sensor.
 *
 * @param ppm_out Pointer to a variable where the CO2 concentration will be stored.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_read_ppm(uint16_t *ppm_out);

/**
 * @brief Calibrate the CO2 sensor using a reference concentration.
 *
 * This function calibrates the sensor against a given reference CO2 concentration (ppm).
 *
 * @param ref_ppm The reference CO2 concentration in ppm.
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_calibrate(uint16_t ref_ppm);

/**
 * @brief Force compensation adjustment on the sensor.
 *
 * This function forces the sensor to perform a compensation adjustment, which might be necessary
 * under certain environmental conditions.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_force_compensation();

/**
 * @brief Enable Auto Baseline Offset Compensation (ABOC).
 *
 * This function enables the sensor's ABOC feature for automatic baseline adjustment.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_enable_aboc();

/**
 * @brief Disable Auto Baseline Offset Compensation (ABOC).
 *
 * This function disables the sensor's ABOC feature.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_disable_aboc();

/**
 * @brief Disable the IIR filter in the sensor.
 *
 * This function disables the sensor's IIR filter, which is used to smooth sensor readings.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_disable_iir();

/**
 * @brief Enable the IIR filter in the sensor.
 *
 * This function enables the sensor's IIR filter to smooth sensor readings.
 *
 * @return esp_err_t ESP_OK on success, or an error code on failure.
 */
esp_err_t co2sensor_enable_iir();

/**
 * @brief Wait for the sensor to become ready for a new measurement.
 *
 * This function polls the sensor's status until it is ready or until a specified timeout occurs.
 *
 * @param timeout_ms Maximum time to wait in milliseconds.
 * @return esp_err_t ESP_OK if the sensor becomes ready, or an error code on failure or timeout.
 */
esp_err_t co2sensor_wait_for_ready(int timeout_ms);

#endif // CO2SENSOR_H
