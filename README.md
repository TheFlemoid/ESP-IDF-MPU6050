# ESP-IDF MPU6050

<H2>ESP-32 drivers for MPU6050 IMUs.</H2>
The MPU6050 is a cheap, powerful, single board, I2C controlled IMU module that incorporates seven separate sensors: a three axis accelerometer, a three access gyroscope, and a temperature sensor.  Other ESP-IDF component libraries for this sensor [exist](https://github.com/MianIdrees/mpu6050_interfacing_with_ESP32_using_ESP-IDF/tree/main) and are very feature complete, however all of the ones I was able to find unfortunately use ESP-IDFs older I2C drivers instead of the new ones.  

&nbsp;

Somewhat recently (late 2024 I believe) Espressif added a second set of newer I2C drivers to ESP-IDF, due to their belief that the first version I2C driver had some inherent issues and wasn't easy to use ([see here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html)).  You can still use the old I2C drivers, and they still include them with ESP-IDF, however there is a stipulation that a project (and any components being used in that project) can only use one type of the driver.  A single project can't contain both the old and the new I2C drivers.

&nbsp;

I was recently working on project where I wanted to use an MPU6050, but I also wanted to use some ESP-IDF I2C based components that had been written with the new I2C drivers.  I was unable to find MPU6050 support using the new I2C drivers, so I wrote my own controller in my project instead of using an external component.  Having accomplished all that, I decided to go ahead and build it into this component myself with the hopes that others may have an easier time with this powerful module going forward.

&nbsp;

Licensed under the permissive MIT License, these drivers can be copied, used, modified, and distributed openly (even for commercial work).  Attribution and/or bug fixed upstreaming is appreciated, but is in no way mandatory.

---

<H3>What this driver can currently do:</H3>
 - Poll and wake up an MPU6050 IMU using either the default MPU6050 address of 0x68 or the alternate address of 0x69.
 - Configure that MPU6050 module for use, configuring the accelerometer sensitivity, the gyrosope sensitivity, and the sample rate.
   - Note that there are some stipulations about this, which I will discuss below.
 - Calibrate the accelerometer and the gyroscope sensors.  This is done during initialization (can be turned off if you'd prefer to do it once and just apply those values yourself, as it extends startup time by around 2 seconds, and requires holding the module level and steady during initialization time).
 - Get readings for all three axes of the accelerometer (in units of Earth gravitational constants, or Gs), all three axes of the gyroscope (in units of degrees per second), and the thermometer (in units of degrees celcius).

 <H3>What this driver cannot currently do:</H3>
  - Regarding the stipulation about MPU6050 initialization alluded to above, the MPU6050 can set the gyroscope to multiple separate sensitivities for both the accelerometer and the gyroscope.  I don't presently support this, and instead just always initialize the accelerometer to +/-2G sensitivity and the gyroscope to +/-250dps.  These are the most accurate readings for slow speed application, and they're what I need for the project I'm working on.  Variation here would be trivial to add, and I may just do it anyway for fun, but while writing this I'm basically just setting up the basic component to accomplish what I need.
  - Similarly, the unit supports multiple sensor reading frequencies.  I always set the sensor reading frequency to 250Hz (a new reading every 4ms), even though we may not end up needing that reading.  I could also change this, but it would be a little more involved as it would mess with the calibration sequence.
  - When using this sensor, it's not typical to use the values that it provides directly.  Instead it's more common to do sensor integration through a complementary filter or a Kalman filter, and track pitch and roll more accurately then the sensor can individually provide accelerometer and gyro values.  I'd like to implmenet this in the library directly, and that's what I intend to do with this library going forward.

<H3> Driver Usage:</H3>
As stated above, this driver assumes you're using the ESP-IDF [i2c_master_controller](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html#i2c_master_controller) driver provided by Espressif.  So, before 
