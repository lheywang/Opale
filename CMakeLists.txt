# Defaults CMake config...
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(Opale)

# ============
# INCLUDES
# ============
# Remove the requirement for absolute include path
target_include_directories(app PUBLIC application/include)

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
target_sources(app PRIVATE application/src/drivers/bno055/bno055.cpp)
## Teseo
target_sources(app PRIVATE application/src/drivers/teseo/teseo.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibGNSS/gnss_data.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibGNSS/gnss_parser.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibNMEA/NMEA_parser.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibGNSS1A1/gnss1a1_gnss.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibGNSS1A1/gnss_lib_config.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibTESEO/teseo_liv3f_i2c.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibTESEO/teseo_liv3f_queue.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibTESEO/teseo_liv3f_uart.cpp)
target_sources(app PRIVATE application/src/drivers/teseo/LibTESEO/teseo_liv3f.cpp)
## II2SDLPC
target_sources(app PRIVATE application/src/drivers/iis2dlpc/iis2dlpc_reg.cpp)
target_sources(app PRIVATE application/src/drivers/iis2dlpc/accelerometer.cpp)
## MCP23008
target_sources(app PRIVATE application/src/drivers/mcp23008/mcp23008.cpp)
## MS5611
target_sources(app PRIVATE application/src/drivers/ms5611/MS5611.cpp)

# ============
# THREADS
# ============
target_sources(app PRIVATE application/src/threads/controller/controller.cpp)
target_sources(app PRIVATE application/src/threads/logger/logger.cpp)
target_sources(app PRIVATE application/src/threads/safety/safety.cpp)
target_sources(app PRIVATE application/src/threads/measure/measure.cpp)

