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
target_sources(app PRIVATE application/src/devices/eeprom/eeprom.cpp)

# ============
# DRIVERS
# ============
## BNO055
target_sources(app PRIVATE application/src/drivers/BNO055/bno055.cpp)
## Teseo

## II2SDLPC
target_sources(app PRIVATE application/src/drivers/IIS2DLPC/iis2dlpc_reg.cpp)
target_sources(app PRIVATE application/src/drivers/IIS2DLPC/accelerometer.cpp)
## MCP23008
target_sources(app PRIVATE application/src/drivers/MCP23008/mcp23008.cpp)
## MS5611
target_sources(app PRIVATE application/src/drivers/MS5611/MS5611.cpp)

