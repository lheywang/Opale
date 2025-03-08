# Defaults CMake config...
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(Opale)

# ============
# MAIN
# ============
target_sources(app PRIVATE application/src/main.c)

# ============
# INIT
# ============
target_sources(app PRIVATE application/src/init/init.c)

# ============
# PERIPHERALS
# ============
target_sources(app PRIVATE application/src/peripherals/gpio/gpio.c)
target_sources(app PRIVATE application/src/peripherals/saadc/saadc.c)

# ============
# DEVICES
# ============
target_sources(app PRIVATE application/src/devices/servo/servo.c)
target_sources(app PRIVATE application/src/devices/rgb/rgb.c)
target_sources(app PRIVATE application/src/devices/rocket/rocket.c)

# ============
# DRIVERS
# ============
## BNO055
target_sources(app PRIVATE application/src/drivers/bno055/bno055.c)
target_sources(app PRIVATE application/src/drivers/bno055/imu.c)
## Teseo
target_sources(app PRIVATE application/src/drivers/teseo/teseo.c)
## II2SDLPC
target_sources(app PRIVATE application/src/drivers/iis2dlpc/ii2sdlpc_reg.c)
target_sources(app PRIVATE application/src/drivers/iis2dlpc/accelerometer.c)
## MCP23008
target_sources(app PRIVATE application/src/drivers/mcp23008/mcp23008.c)


# Random, for future...
target_sources(app PRIVATE application/src/devices/eeprom.c)
target_sources(app PRIVATE application/src/devices/barometer.c)
target_sources(app PRIVATE application/src/devices/gps.c)
