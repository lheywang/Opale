# Defaults CMake config...
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(Opale)

# ============
# MAIN
# ============
target_sources(app PRIVATE application/src/main.cpp)

# ============
# INIT
# ============
target_sources(app PRIVATE application/src/init/init.cpp)

# ============
# PERIPHERALS
# ============
target_sources(app PRIVATE application/src/peripherals/gpio/gpio.cpp)
target_sources(app PRIVATE application/src/peripherals/saadc/saadc.cpp)

# ============
# DEVICES
# ============
target_sources(app PRIVATE application/src/devices/servo/servo.cpp)
target_sources(app PRIVATE application/src/devices/rgb/rgb.cpp)
target_sources(app PRIVATE application/src/devices/rocket/rocket.cpp)

# ============
# DRIVERS
# ============
## BNO055
target_sources(app PRIVATE application/src/drivers/bno055/bno055.cpp)
target_sources(app PRIVATE application/src/drivers/bno055/imu.cpp)
## Teseo

## II2SDLPC
target_sources(app PRIVATE application/src/drivers/iis2dlpc/iis2dlpc_reg.cpp)
target_sources(app PRIVATE application/src/drivers/iis2dlpc/accelerometer.cpp)
## MCP23008
target_sources(app PRIVATE application/src/drivers/mcp23008/mcp23008.cpp)

# Random, for future...
target_sources(app PRIVATE application/src/devices/eeprom.cpp)
target_sources(app PRIVATE application/src/devices/barometer.cpp)
target_sources(app PRIVATE application/src/devices/gps.cpp)
