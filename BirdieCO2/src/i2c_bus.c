#include "i2c_bus.h"    // Include header file that declares I2C bus configurations and prototypes.
#include "esp_log.h"    // Include ESP-IDF logging library for logging debug/info messages.

// Tag used for logging messages related to the I2C bus.
static const char *TAG_I2C_BUS = "I2C_BUS";

/**
 * @brief Initialize the I2C master driver.
 *
 * This function sets up the I2C configuration structure with the proper settings such as mode,
 * SDA/SCL pin numbers, pull-up configuration, and clock frequency. It then applies the configuration
 * and installs the I2C driver.
 *
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_master_init(void) {
    // Configure I2C settings
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,                  // Set I2C to master mode.
        .sda_io_num = I2C_MASTER_SDA_IO,            // Assign the SDA GPIO pin.
        .sda_pullup_en = GPIO_PULLUP_DISABLE,       // Optionally enable/disable internal pull-up resistor for SDA.
        .scl_io_num = I2C_MASTER_SCL_IO,            // Assign the SCL GPIO pin.
        .scl_pullup_en = GPIO_PULLUP_DISABLE,       // Optionally enable/disable internal pull-up resistor for SCL.
        .master.clk_speed = I2C_MASTER_FREQ_HZ,     // Set clock frequency for I2C communications.
    };

    // Apply the I2C configuration to the specified I2C port.
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));

    // Log the installation configuration: SDA pin, SCL pin, and clock frequency.
    ESP_LOGI(TAG_I2C_BUS, "I2C driver installed on SDA=%d, SCL=%d, freq=%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    // Install the I2C driver on the specified I2C port in master mode.
    // The last three parameters are for RX buffer, TX buffer, and flags (set to 0 here).
    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

/**
 * @brief Scan the I2C bus for connected devices.
 *
 * This function iterates through all possible 7-bit I2C addresses (1 to 126)
 * and attempts a write operation to detect if a device acknowledges the transfer.
 * If a device responds with an ESP_OK status, it is logged as found.
 */
void i2c_scan(void)
{
    // Log the beginning of the I2C scan process.
    ESP_LOGI(TAG_I2C_BUS, "Scanning I2C bus on port %d...", I2C_MASTER_NUM);

    // Loop through all possible 7-bit addresses (excluding reserved addresses).
    for (uint8_t addr = 1; addr < 127; addr++) {
        // Create a new I2C command link.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        // Initiate the start condition.
        i2c_master_start(cmd);
        // Send the device address shifted left with the write bit appended.
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        // Issue a stop condition.
        i2c_master_stop(cmd);
        // Execute the command link with a timeout (50 ms).
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        // Free the command link to release allocated resources.
        i2c_cmd_link_delete(cmd);

        // If the command returned ESP_OK, a device acknowledged the transfer.
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_I2C_BUS, "Found I2C device at 0x%02X", addr);
        }
    }
    // Log the completion of the I2C scan.
    ESP_LOGI(TAG_I2C_BUS, "I2C scan done.");
}

/**
 * @brief Write a sequence of bytes to a specific register of an I2C device.
 *
 * This function writes one or more bytes starting at the specified register address of a device
 * identified by its I2C address.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address within the device to write to.
 * @param data Pointer to the data buffer containing the bytes to write.
 * @param len The number of bytes to write.
 *
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    // Create a new I2C command link.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Start the I2C command sequence.
    i2c_master_start(cmd);
    // Send the device address with the write bit.
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    // Write the register address to which the data will be written.
    i2c_master_write_byte(cmd, reg_addr, true);
    // Write the specified number of data bytes.
    i2c_master_write(cmd, data, len, true);
    // Stop the I2C command sequence.
    i2c_master_stop(cmd);
    // Execute the command link with a timeout (specified by I2C_TIMEOUT_MS).
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    // Delete the command link to free resources.
    i2c_cmd_link_delete(cmd);
    // Return the result of the write operation.
    return ret;
}

/**
 * @brief Read a sequence of bytes from a specific register of an I2C device.
 *
 * This function performs a read operation from a device starting at the given register.
 * It first writes the register address, then restarts the I2C sequence to read the requested
 * number of bytes from the device.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address within the device from where data is to be read.
 * @param data Pointer to the buffer where the read bytes will be stored.
 * @param len The number of bytes to read.
 *
 * @return esp_err_t Returns ESP_OK on success, or an error code on failure.
 */
esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len) {
    // Create a new I2C command link.
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Start the I2C command sequence.
    i2c_master_start(cmd);
    // Send the device address with the write bit to specify the register.
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    // Write the register address from which data will be read.
    i2c_master_write_byte(cmd, reg_addr, true);

    // Restart the I2C command sequence for the read operation.
    i2c_master_start(cmd);
    // Send the device address with the read bit.
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);
    // Read the specified number of bytes.
    // The I2C_MASTER_LAST_NACK flag sends a NACK after the final byte to end the reading sequence.
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    // Issue the stop condition.
    i2c_master_stop(cmd);

    // Execute the command link with a timeout (specified by I2C_TIMEOUT_MS).
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    // Delete the command link to free resources.
    i2c_cmd_link_delete(cmd);
    // Return the result of the read operation.
    return ret;
}

// 16-bit big-endian read
/**
 * @brief Read a 16-bit big-endian value from an I2C device register.
 *
 * This function reads two bytes from the specified register of the I2C device,
 * treating the first byte as the high byte and the second as the low byte,
 * then combines them into a single 16-bit unsigned integer.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address to read from.
 * @param out Pointer to a 16-bit variable where the result will be stored.
 * @return esp_err_t Returns ESP_OK on success or an error code on failure.
 */
esp_err_t i2c_read_u16_be(uint8_t dev_addr, uint8_t reg_addr, uint16_t *out) {
    uint8_t buf[2];  // Buffer to hold the two bytes read from the device.
    
    // Use the generic I2C read function to obtain 2 bytes from the specified register.
    esp_err_t ret = i2c_read_bytes(dev_addr, reg_addr, buf, 2);
    
    // If the read operation succeeded...
    if (ret == ESP_OK) {
        // Combine the two bytes into a 16-bit number.
        // buf[0] is treated as the high byte and buf[1] as the low byte (big-endian format).
        *out = (buf[0] << 8) | buf[1];
    }
    
    // Return the result code from the I2C read operation.
    return ret;
}

// 16-bit big-endian write
/**
 * @brief Write a 16-bit big-endian value to an I2C device register.
 *
 * This function splits a 16-bit unsigned integer into two bytes,
 * arranging them in big-endian order (high byte first, low byte second),
 * and writes them to the specified register of the I2C device.
 *
 * @param dev_addr The 7-bit I2C address of the device.
 * @param reg_addr The register address to which the data will be written.
 * @param val The 16-bit unsigned integer to write.
 * @return esp_err_t Returns ESP_OK on success or an error code on failure.
 */
esp_err_t i2c_write_u16_be(uint8_t dev_addr, uint8_t reg_addr, uint16_t val) {
    // Prepare a two-byte buffer:
    // The high byte (most significant 8 bits) is stored in buf[0],
    // and the low byte (least significant 8 bits) is stored in buf[1].
    uint8_t buf[2] = { (val >> 8) & 0xFF, val & 0xFF };
    
    // Use the generic I2C write function to write the two-byte buffer to the device.
    return i2c_write_bytes(dev_addr, reg_addr, buf, 2);
}
