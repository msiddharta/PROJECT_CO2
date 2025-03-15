#include "co2sensor.h"
#include "esp_log.h"
#include "driver/gpio.h"

// I2C Write Function
static esp_err_t i2c_master_write_to_sensor(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PASCO2_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// I2C Read Function
static esp_err_t i2c_master_read_sensor(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PASCO2_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (PASCO2_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Initialize I2C
static void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// Outside functions calling inside ("private") functions
void co2sensor_init() {
    // Initialize I2C and sensor
    i2c_master_init();
    printf("CO2 Sensor initialized.\n");
}

void co2sensor_calibrate(uint16_t ref_co2_ppm) {
    calibrate_sensor(ref_co2_ppm);
}

void co2sensor_enable_aboc() {
    enable_aboc();
}

bool co2sensor_is_data_ready() {
    return is_data_ready();
}

uint16_t co2sensor_read_co2() {
    uint8_t co2_high, co2_low;
    uint16_t co2_ppm = 0;

    if (i2c_master_read_sensor(REG_CO2PPM_H, &co2_high, 1) == ESP_OK &&
        i2c_master_read_sensor(REG_CO2PPM_L, &co2_low, 1) == ESP_OK) {
        co2_ppm = ((int)co2_high << 8) | co2_low;  // Combine the high and low byte
    } else {
        printf("Error reading CO2 concentration.\n");
    }

    return co2_ppm;
}

// Scan I2C Bus for Devices
void i2c_scan() {
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("Found device at address: 0x%X\n", addr);
        }
    }
}

// Trigger a Single Measurement
void trigger_single_measurement() {
    uint8_t mode = 0x01;  // Single-shot mode = 0x01, continuous= 0x10
    esp_err_t ret = i2c_master_write_to_sensor(REG_MEAS_CFG, &mode, 1);
    if (ret == ESP_OK) {
        printf("Measurement triggered\n");
    } else {
        printf("Failed to trigger measurement\n");
    }


    // Wait for the measurement to complete (with a timeout)
    int retries = 10;  // Retry count (for example, 10 retries)
    while (retries--) {
        uint8_t status;
        if (i2c_master_read_sensor(REG_MEAS_STS, &status, 1) == ESP_OK) {
            printf("Status Register: 0x%02X\n", status);  // Debugging line
            if (status & (1 << 4)) {  // Check DRDY (bit 4)
                printf("Data is ready!\n");
                return;  // Data is ready, exit
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait 500ms before retrying
    }
    printf("Data not ready after timeout\n");
}

bool is_data_ready() {
    uint8_t status;
    if (i2c_master_read_sensor(REG_MEAS_STS, &status, 1) == ESP_OK) {
        printf("Status Register: 0x%02X\n", status);  // Debugging line
        // Check DRDY bit (bit 4)
        return (status & (1 << 4)) != 0;
    }
    return false;
}

void wait_for_data_ready() {
    int retries = 10;  // Retry up to 10 times
    while (retries--) {
        if (is_data_ready()) {
            printf("Data is ready!\n");
            return;  // Data is ready, exit
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);  // Wait 500ms before retrying
    }
    printf("Data not ready after timeout\n");
}

// Calibrate the Sensor
void calibrate_sensor(uint16_t ref_co2_ppm) {
    // Ensure the reference value is within the valid range (350 ppm to 1500 ppm)
    if (ref_co2_ppm < 350) {
        ref_co2_ppm = 350;
    } else if (ref_co2_ppm > 1500) {
        ref_co2_ppm = 1500;
    }

    // Split the reference value into two bytes (MSB and LSB)
    uint8_t calib_high = (ref_co2_ppm >> 8) & 0xFF;  // High byte (MSB)
    uint8_t calib_low = ref_co2_ppm & 0xFF;          // Low byte (LSB)

    // Write the calibration values to the sensor
    esp_err_t ret1 = i2c_master_write_to_sensor(REG_CALIB_REF_H, &calib_high, 1);
    esp_err_t ret2 = i2c_master_write_to_sensor(REG_CALIB_REF_L, &calib_low, 1);

    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        printf("Calibration set to %d ppm\n", ref_co2_ppm);
    } else {
        printf("Failed to set calibration\n");
    }

    // Apply Calibration (Save to Non-Volatile Memory)
    uint8_t save_cmd = 0xCF;  // Save forced calibration command
    esp_err_t ret3 = i2c_master_write_to_sensor(REG_SENS_RST, &save_cmd, 1);
    if (ret3 == ESP_OK) {
        printf("Calibration saved!\n");
    } else {
        printf("Failed to save calibration!\n");
    }    
}

// Enable ABOC (Automatic Baseline Correction)
void enable_aboc() {
    uint8_t aboc_enable = 0x01;  // Enable ABOC
    esp_err_t ret = i2c_master_write_to_sensor(REG_MEAS_CFG, &aboc_enable, 1);
    if (ret == ESP_OK) {
        printf("ABOC enabled (Automatic Baseline Correction)\n");
    } else {
        printf("Failed to enable ABOC\n");
    }
}