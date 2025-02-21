# Defaults CMake config...
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(Opale)

# Main source
target_sources(app PRIVATE application/src/main.c)

# Initializer
target_sources(app PRIVATE application/src/init/init.c)

# Devices
target_sources(app PRIVATE application/src/devices/servo.c)
target_sources(app PRIVATE application/src/devices/rgb.c)
target_sources(app PRIVATE application/src/devices/gpio.c)
target_sources(app PRIVATE application/src/devices/eeprom.c)
target_sources(app PRIVATE application/src/devices/expander.c)
target_sources(app PRIVATE application/src/devices/barometer.c)
target_sources(app PRIVATE application/src/devices/accelerometer.c)
target_sources(app PRIVATE application/src/devices/gps.c)
target_sources(app PRIVATE application/src/devices/imu.c)
