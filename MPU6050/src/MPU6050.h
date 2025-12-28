/**
 * File:       MPU6050.h
 * Author:     Franklyn Dahlberg
 * Created:    26 December, 2025
 * Copyright:  2025 (c) Franklyn Dahlberg
 * License:    MIT License (see http://choosealicense.com/licenses/mit/)
 */

/**
 * Header for MPU6050 ESP-IDF component using latest I2C library
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c_master.h"

#define MPU6050_PWR_MGMT_1          0x6b         // MPU6050 pwr mgmt register
#define MPU6050_SMPLRT_DIV          0x19         // Register to set the sample rate
#define MPU6050_GYRO_CONFIG         0x1b         // Register to set the gyro sensitivity
#define MPU6050_ACCEL_CONFIG        0x1c         // Register to set the acceleration sensitivity
#define MPU6050_X_ACCEL_START_ADDR  0x3b         // MSB register for X acceleration, the start of the accel registers
#define MPU6050_TEMP_START_ADDR     0x41         // MSB register for temp, the start of the temp registers
#define MPU6050_X_GYRO_START_ADDR   0x43         // MSB register for X gyro, the start of the gyro registers

#define ACCEL_RAW_DATA_CONST        16384.0      // Constant to divide the acceleration raw value by to get the result in Gs
#define GYRO_RAW_DATA_CONST         134.0        // Constant to divde the gyro value by to get the result in degrees per second
#define CAL_READINGS                200          // Readings to average during sensor calibration

/**
 * Enum used to describe the orientation that the accelerometer should be calibrated to
 * (ie. the module direction perpendicular to the earths surface at initialization)
 */
typedef enum _cal_axis {
    X,
    Y,
    Z
} CAL_AXIS;

/**
 * Struct containing the definition of an MPU6050 I2C sensor.
 */
typedef struct _mpu6050Config {
    char sensor_address;
    bool should_calibrate;
    CAL_AXIS calibration_axis;
} MPU6050_CONFIG;

// Function declaration
// Functions intended for external use
void setup_mpu_sensor(i2c_master_bus_handle_t *i2c_bus_handle, MPU6050_CONFIG *mpu_config);
void read_accel(double* accel);
void read_temp(double* temp);
void read_gyro(double* gyro);

// Functions intended for internal use
void read_accel_raw(int16_t* accel_raw);
void read_gyro_raw(int16_t* gyro_raw);
void calibrate_module(CAL_AXIS cal_axis);
