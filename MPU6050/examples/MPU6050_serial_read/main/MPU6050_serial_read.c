/**
 * File:       MPU6050_serial_read.c
 * Author:     Franklyn Dahlberg
 * Created:    25 December, 2025
 * Copyright:  2025 (c) Franklyn Dahlberg
 * License:    MIT License (see http://choosealicense.com/licenses/mit/)
 */

/**
 * Example application to configure an MPU-6050, calibrate it, and output the IMU data to serial.
 */
#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "MPU6050.h"

#define MPU6050_SENSOR_ADDRESS   0x68
#define I2C_MASTER_SDA_IO        21
#define I2C_MASTER_SCL_IO        22

// Global vars
i2c_master_bus_handle_t i2cBusHandle;
double accelReadings[3];
double gyroReadings[3];
double tempReading;

static uint32_t ONE_HUNDRED_MILLI_DELAY = (100 / portTICK_PERIOD_MS);

/**
 * Application main
 */
void app_main(void) {

    // Setup the I2C bus, with GPIO 21 as SDA and GPIO 22 as SCL
    i2c_master_bus_config_t i2cConfig = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cConfig, &i2cBusHandle));

    // Make an MPU6050_CONFIG object, with 0x68 as the I2C address, calibrating,
    // with the Z-axis facing down.
    MPU6050_CONFIG mpuConfig = {
        .sensor_address = MPU6050_SENSOR_ADDRESS,
        .should_calibrate = true,
        .calibration_axis = Z
    };
    setup_mpu_sensor(&i2cBusHandle, &mpuConfig);

    // In a while loop, read the sensor and print the readings
    while (1) {
        read_accel(accelReadings);
        read_temp(&tempReading);
        read_gyro(gyroReadings);
        printf("Accel: x: %0.3f, y: %0.3f, z: %0.3f   Temp: %0.3f  Gyro: x: %0.3f, y: %0.3f, z: %0.3f\n", 
                    accelReadings[0], accelReadings[1], accelReadings[2], tempReading, gyroReadings[0], 
                    gyroReadings[1], gyroReadings[2]);
        vTaskDelay(ONE_HUNDRED_MILLI_DELAY);
    }
}

