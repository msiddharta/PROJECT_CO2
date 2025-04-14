#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c.h"

/**
 * @brief I2C port and pin configuration.
 *
 * These macros define the I2C port number, the GPIO pins for SCL and SDA,
 * the operating frequency, and the timeout (in milliseconds) for I2C operations.
 */
#define I2C_MASTER_NUM         I2C_NUM_0     // I2C port number used by the master.
#define I2C_MASTER_SCL_IO      4             // GPIO pin for I2C SCL signal.
#define I2C_MASTER_SDA_IO      3             // GPIO pin for I2C SDA signal.
#define I2C_MASTER_FREQ_HZ     400000        // I2C clock frequency in Hertz.
#define I2C_TIMEOUT_MS         1000          // Timeout duration for I2C operations in milliseconds.

/**
 * @brief Initializes the I2C master driver.
 *
 * This function sets up the I2C bus with the specified configuration.
 * It configures the GPIO pins, the I2C mode, and installs the driver.
 *
 * @return esp_err_t Returns ESP_OK on success, or the appropriate error code.
 */
esp_err_t i2c_master_init(void);

/**
 * @brief Scans the I2C bus for connected devices.
 *
 * This function iterates through possible I2C addresses and identifies
 * any devices that are present on the bus.
 */
void i2c_scan(void);

/**
 * @brief Writes data to a specified register of an I2C device.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address to write to.
 * @param data Pointer to the data buffer to send.
 * @param len Number of bytes in the data buffer.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Reads data from a specified register of an I2C device.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address to read from.
 * @param data Pointer to the data buffer to store the read data.
 * @param len Number of bytes to read.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Reads a 16-bit big-endian value from an I2C device register.
 *
 * The two bytes are read from the specified register and combined in big-endian order.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address to read from.
 * @param out Pointer to a 16-bit variable to store the combined value.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_read_u16_be(uint8_t dev_addr, uint8_t reg_addr, uint16_t *out);

/**
 * @brief Writes a 16-bit big-endian value to an I2C device register.
 *
 * The 16-bit value is split into two bytes in big-endian order and written to the register.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address to write to.
 * @param val The 16-bit value to write.
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_write_u16_be(uint8_t dev_addr, uint8_t reg_addr, uint16_t val);

#endif // I2C_BUS_H
