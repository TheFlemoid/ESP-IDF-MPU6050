/**
 * File:       MPU6050.c
 * Author:     Franklyn Dahlberg
 * Created:    19 December, 2025
 * Copyright:  2025 (c) Franklyn Dahlberg
 * License:    MIT License (see http://choosealicense.com/licenses/mit/)
 */

/**
 * ESP-IDF component to read/control MPU-6050 sensors via the "new" (circa 2024) ESP-IDF
 * I2C interface.
 */
#include <stdio.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "MPU6050.h"

#define I2C_MASTER_SCL_IO           22           // SCL pin
#define I2C_MASTER_SDA_IO           21           // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0    // ESP I2C bus
#define I2C_MASTER_FREQ_HZ          400000       // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0            // No I2C master tx buffer
#define I2C_MASTER_RX_BUF_DISABLE   0            // No I2C master rx buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// Global vars
i2c_master_bus_config_t i2cConfig;  // Master I2C config, generated and initialized by component user
uint8_t reg_data[2];                // Used during I2C comms, kept on the heap for memory management
uint8_t read_reg;                   // Used during I2C comms, kept on the heap for memory management
double accel[3];                    // Used to pass accelerometer readings around
double temp;                        // Used to pass temp readings around
double gyro[3];                     // Used to pass gyro readings around
bool calibration_done = false;      // Global flag for whether calibration has been performed
int16_t accel_correction[3];        // Used to correct the accelerometer reading
int16_t gyro_correction[3];         // Used to correct the gyroscope reading

//i2c_master_bus_config_t i2cConfig = {
//    .clk_source = I2C_CLK_SRC_DEFAULT,
//    .scl_io_num = I2C_MASTER_SCL_IO,
//    .sda_io_num = I2C_MASTER_SDA_IO,
//    .glitch_ignore_cnt = 7,
//    .flags.enable_internal_pullup = true
//};

MPU6050_CONFIG mpuConfig;
i2c_master_bus_handle_t i2cBusHandle;
i2c_master_dev_handle_t mpuSensorHandle;

static uint32_t ONE_HUNDRED_MILLI_DELAY = (100 / portTICK_PERIOD_MS);
static uint32_t TEN_MILLI_DELAY = (10 / portTICK_PERIOD_MS);

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

/**
 * Sets up the MPU6050 sensor for operations.
 */
void setup_mpu_sensor(i1c_master_bus_handle_t i2c_bus_handle, MPU6050_Config mpu_config, 
        bool should_cal, enum Cal_Axis cal_axis) {

    i2cBusHandle = i2c_bus_handle;
    ESP_ERROR_CHECK(i2c_master_probe(i2c_bus_handle, mpu_config->sensor_address, -1));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBusHandle, &mpu6050Config, &mpuSensorHandle));

    // The sensor powers on in "sleep mode", so this "wakes up" the sensor to place 
    // it in measure mode.
    uint8_t monitor_cmd[2]; 
    monitor_cmd[0] = MPU6050_PWR_MGMT_1; 
    monitor_cmd[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, monitor_cmd, 2, -1));

    // Configure the sensors gyro for +/- 250dps mode (the most accurate)
    monitor_cmd[0] = MPU6050_GYRO_CONFIG;
    monitor_cmd[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, monitor_cmd, 2, -1));

    // Configure the sensors accelerometer for +/- 2g mode (the most accurate)
    monitor_cmd[0] = MPU6050_ACCEL_CONFIG;
    monitor_cmd[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, monitor_cmd, 2, -1));

    // Set the sensors sample rate to 100Hz (1kHz / (9 + 1))
    monitor_cmd[0] = MPU6050_SMPLRT_DIV;
    monitor_cmd[1] = 0x09;
    ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, monitor_cmd, 2, -1));

    if (should_cal) {
        calibrate_module(cal_axis);
    }

    printf("MPU6050 initialized and ready for sampling.\n");
}

/**
 * Reads the linear acceleration values from the MPU-6050 and includes the values in the param double array.
 *
 * @param accel three element double[3] array to be filled out (index 0=x, 1=y, 2=z)
 */
void read_accel(double* accel) {
    int16_t raw_readings[3];
    read_accel_raw(raw_readings);

    // If we've performed module calibration, apply the correction
    if (calibration_done) {
        for (int i = 0; i < 3; i++) {
            raw_readings[i] -= accel_correction[i];
        }
    }

    for(int i = 0; i < 3; i++) {
        accel[i] = raw_readings[i] / (double)ACCEL_RAW_DATA_CONST;
    }
}

/**
 * Reads the raw accelerometer readings from the MPU-6050 and includes them in the param three index int16_t array.
 * NOTE: This helper function is broken out to allow for use during calibration.
 *
 * @param accel_raw three element int16_t array to be filled out (index 0=x, 1=y, 2=z)
 */
void read_accel_raw(int16_t* accel_raw) {
    // Write and read both registers
    for (int i = 0; i < 6; i+=2) {
        for (int j = 0; j < 2; j++) {
            read_reg = MPU6050_X_ACCEL_START_ADDR + i + j;
            ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, &read_reg, 1, -1));
            ESP_ERROR_CHECK(i2c_master_receive(mpuSensorHandle, &reg_data[j], 1, -1));
        }
        accel_raw[i / 2] = (reg_data[0] << 8) | reg_data[1];
    }
}

/**
 * Reads the temperature value from the MPU-6050 and includes the value in the param double pointer.
 *
 * @param temp int16_t pointer to be filled out
 */
void read_temp(double* temp) {
    // Write and read both registers
    for (int i = 0; i < 2; i++) {
        read_reg = MPU6050_TEMP_START_ADDR + i;
        ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, &read_reg, 1, -1));
        ESP_ERROR_CHECK(i2c_master_receive(mpuSensorHandle, &reg_data[i], 1, -1));
    }

    int16_t temp_raw = (reg_data[0] << 8) | reg_data[1];
    *temp = (double)(temp_raw / 340.0 + 36.53);
}

/**
 * Reads the gyro values from the MPU-6050 and includes the values in the param double array.
 *
 * @param gyro three element double[3] array to be filled out (index 0=x, 1=y, 2=z)
 */
void read_gyro(double* gyro) {
    int16_t raw_readings[3];
    read_gyro_raw(raw_readings);

    // If we've performed module calibration, apply the correction
    if (calibration_done) {
        for (int i = 0; i < 3; i++) {
            raw_readings[i] -= gyro_correction[i];
        }
    }

    for (int i = 0; i < 3; i++) {
        gyro[i] = raw_readings[i] / (double)GYRO_RAW_DATA_CONST;
    }
}

/**
 * Reads the raw gyro readings from the MPU-6050 and includes them in the param three index int16_t array.
 * NOTE: This helper function is broken out to allow for use during calibration.
 *
 * @param gyro_raw three element int16_t array to be filled out (index 0=x, 1=y, 2=z)
 */
void read_gyro_raw(int16_t* gyro_raw) {
    // Write and read both registers
    for (int i = 0; i < 6; i+=2) {
        for (int j = 0; j < 2; j++) {
            read_reg = MPU6050_X_GYRO_START_ADDR + i + j;
            ESP_ERROR_CHECK(i2c_master_transmit(mpuSensorHandle, &read_reg, 1, -1));
            ESP_ERROR_CHECK(i2c_master_receive(mpuSensorHandle, &reg_data[j], 1, -1));
        }
        gyro_raw[i / 2] = (reg_data[0] << 8) | reg_data[1];
    }
}

/**
 * Calibrates the module, should be performed prior to use.
 * NOTE: This function expects that the module is oriented such that the axis of the module specified
 *       the param enum cal_axis is facing towards the Earth.  This will calibrate that axis so it should
 *       be set to 1g during cal, and the other axes set to 0g.  This should be performed while the module
 *       is stationary.
 */
 void calibrate_module(enum Cal_Axis cal_axis) {
    int32_t accel_cals[3] = {0, 0, 0};
    int32_t gyro_cals[3] = {0, 0, 0};

    printf("Calibrating the MPU-6050 module.  The module should be held steady and upright during this process.\n");

    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    for (int i = 0; i < CAL_READINGS; i++) {
        read_accel_raw(accel_raw);
        read_gyro_raw(gyro_raw);

        for(int j = 0; j < 3; j++) {
            accel_cals[j] += accel_raw[j];
            gyro_cals[j] += gyro_raw[j];
        }
        vTaskDelay(TEN_MILLI_DELAY);
    }

    // Determine the acceleration correction, keeping note of which axis is facing the Earth
    switch(cal_axis) {
        case X:
            accel_correction[0] = (accel_cals[0] / CAL_READINGS) - (int)ACCEL_RAW_DATA_CONST;
            accel_correction[1] = accel_cals[1] / CAL_READINGS;
            accel_correction[2] = accel_cals[2] / CAL_READINGS;
            break;
        case Y:
            accel_correction[0] = accel_cals[0] / CAL_READINGS;
            accel_correction[1] = (accel_cals[1] / CAL_READINGS) - (int)ACCEL_RAW_DATA_CONST;
            accel_correction[2] = accel_cals[2] / CAL_READINGS;
            break;
        case Z:
            accel_correction[0] = accel_cals[0] / CAL_READINGS;
            accel_correction[1] = accel_cals[1] / CAL_READINGS;
            accel_correction[2] = (accel_cals[2] / CAL_READINGS) - (int)ACCEL_RAW_DATA_CONST;
            break;
    }

    // Determine the gyro correction, Earth orientation doesn't matter for these
    for (int i = 0; i < 3; i++) {
        gyro_correction[i] = gyro_cals[i] / CAL_READINGS;
    }

    calibration_done = true;

    printf("Calibration complete.  Accel offsets: x %d, y %d, z %d, x%d, y %d, z %d\n", accel_correction[0], 
           accel_correction[1], accel_correction[2], gyro_correction[0], gyro_correction[1], 
           gyro_correction[2]);
 }
