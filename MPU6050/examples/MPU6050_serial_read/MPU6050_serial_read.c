/**
 * File:       MPU6050.c
 * Author:     Franklyn Dahlberg
 * Created:    25 December, 2025
 * Copyright:  2025 (c) Franklyn Dahlberg
 * License:    MIT License (see http://choosealicense.com/licenses/mit/)
 */

/**
 * Example application to configure an MPU-6050, calibrate it, and output the IMU data to serial.
 */
#include "MPU6050.h"

// Global vars
i2c_master_bus_config_t i2cConfig = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true
};

/**
 * Application main
 */
void app_main(void) {
    setup_i2c();
    setup_mpu_sensor(true, X);

    while (1) {
        read_accel(accel);
        read_temp(&temp);
        read_gyro(gyro);
        printf("Accel: x: %0.3f, y: %0.3f, z: %0.3f   Temp: %0.3f  Gyro: x: %0.3f, y: %0.3f, z: %0.3f\n", 
                    accel[0], accel[1], accel[2], temp, gyro[0], gyro[1], gyro[2]);
        vTaskDelay(ONE_HUNDRED_MILLI_DELAY);
    }
}

/**
 * Initializes the I2C bus using the global I2C config.
 * NOTE: GPIO 21 and 22 are the standard pins for I2C SDA and SCL respectively.
 */
void setup_i2c() {
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cConfig, &i2cBusHandle));
}

