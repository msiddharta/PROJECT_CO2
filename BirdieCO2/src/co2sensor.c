#include "co2sensor.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "i2c_bus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define CO2_ADDR 0x28  // PASCO2 I2C address (7-bit)

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
    uint8_t co2_high = 0, co2_low = 0;
    uint16_t co2_ppm = 0;

    if (i2c_read_bytes(CO2_ADDR, REG_CO2PPM_H, &co2_high, 1) == ESP_OK &&
        i2c_read_bytes(CO2_ADDR, REG_CO2PPM_L, &co2_low, 1) == ESP_OK) {
        co2_ppm = ((uint16_t)co2_high << 8) | co2_low;
    } else {
        printf("Error reading CO2 concentration.\n");
    }

    return co2_ppm;
}

void trigger_single_measurement() {
    uint8_t mode = 0x01;  // Single-shot mode
    esp_err_t ret = i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &mode, 1);
    if (ret == ESP_OK) {
        printf("Measurement triggered\n");
    } else {
        printf("Failed to trigger measurement\n");
    }

    int retries = 10;
    while (retries--) {
        uint8_t status;
        if (i2c_read_bytes(CO2_ADDR, REG_MEAS_STS, &status, 1) == ESP_OK) {
            printf("Status Register: 0x%02X\n", status);
            if (status & (1 << 4)) {
                printf("Data is ready!\n");
                return;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("Data not ready after timeout\n");
}

bool is_data_ready() {
    uint8_t status;
    if (i2c_read_bytes(CO2_ADDR, REG_MEAS_STS, &status, 1) == ESP_OK) {
        printf("Status Register: 0x%02X\n", status);
        return (status & (1 << 4)) != 0;
    }
    return false;
}

void wait_for_data_ready() {
    int retries = 10;
    while (retries--) {
        if (is_data_ready()) {
            printf("Data is ready!\n");
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    printf("Data not ready after timeout\n");
}

void calibrate_sensor(uint16_t ref_co2_ppm) {
    if (ref_co2_ppm < 350) ref_co2_ppm = 350;
    if (ref_co2_ppm > 1500) ref_co2_ppm = 1500;

    uint8_t high = (ref_co2_ppm >> 8) & 0xFF;
    uint8_t low  = ref_co2_ppm & 0xFF;

    esp_err_t ret1 = i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_H, &high, 1);
    esp_err_t ret2 = i2c_write_bytes(CO2_ADDR, REG_CALIB_REF_L, &low, 1);

    if (ret1 == ESP_OK && ret2 == ESP_OK) {
        printf("Calibration set to %d ppm\n", ref_co2_ppm);
    } else {
        printf("Failed to set calibration\n");
    }

    uint8_t save_cmd = 0xCF;
    esp_err_t ret3 = i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &save_cmd, 1);
    if (ret3 == ESP_OK) {
        printf("Calibration saved!\n");
    } else {
        printf("Failed to save calibration!\n");
    }
}

void perform_forced_compensation(void) {
    for (int i = 0; i < 3; i++) {
        trigger_single_measurement();
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    uint8_t save_cmd = 0xCF;
    i2c_write_bytes(CO2_ADDR, REG_SENS_RST, &save_cmd, 1);
    printf("Forced compensation applied and saved!\n");
}

void enable_aboc() {
    uint8_t aboc_enable = 0x01;
    esp_err_t ret = i2c_write_bytes(CO2_ADDR, REG_MEAS_CFG, &aboc_enable, 1);
    if (ret == ESP_OK) {
        printf("ABOC enabled (Automatic Baseline Correction)\n");
    } else {
        printf("Failed to enable ABOC\n");
    }
}

void co2sensor_init(int value) {
    co2sensor_calibrate(value);
    co2sensor_enable_aboc();
    perform_forced_compensation();
}
