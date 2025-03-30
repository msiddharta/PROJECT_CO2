#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "driver/i2c.h"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SCL_IO      4
#define I2C_MASTER_SDA_IO      3
#define I2C_MASTER_FREQ_HZ     400000
#define I2C_TIMEOUT_MS         1000

esp_err_t i2c_master_init(void);
void i2c_scan(void);


esp_err_t i2c_write_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_read_bytes(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t len);
esp_err_t i2c_read_u16_be(uint8_t dev_addr, uint8_t reg_addr, uint16_t *out);
esp_err_t i2c_write_u16_be(uint8_t dev_addr, uint8_t reg_addr, uint16_t val);
#endif // I2C_BUS_H