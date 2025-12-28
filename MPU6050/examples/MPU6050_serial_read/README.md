## ESP-IDF MPU6050 Serial Read Example

An example application showing use of an MPU-6050 module by setting up the module on the
I2C bus, calibrating it, polling the module for sensor readings every 100ms and printing
those readings over the serial UART.  Mostly here to show that the component works, and
to provide an example as to how to use it.

In order to build this project, it must be build with esp-idf from the same directory that
this README is in.  If, like me, you typically compile esp-idf projects in Visual Studio
Code, then you need to open the folder that this README is in from the initial "Open Folder"
dialog, not the root of this repo.  Otherwise, the CMakeList infrastructure of esp-idf
won't work out properly, and you'll end up with a weird precompile error.  Select okay
if/when prompted to generate `compile_commands.json`.

Connect the MPU-6050 module to the ESP-32 as such:

| ESP-32 | MPU-6050 Pin |
| :---: | :---: |
| GPIO 21  | SDA |
| GPIO 22  | SCL |

On the module: 
- VCC should be connected to 3V3 (some modules also allow connecting to 5V through onboard
  power management, but the max voltage for the MPU-6050 chip itself is 3V3 per the datasheet).
- GND should be connected to ground.
- AD0 should be either disconnected or pulled low to ground.

This example runs a module calibration during initialization.  During calibration, the module
must be kept steady (with no rotation on any axis) and with the Z axis facing perpendicular
to the Earth.  For most modules, this means holding it flat against a surface like a table.
