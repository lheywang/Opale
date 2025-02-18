# Defaults CMake config...
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Project name
project(Opale)

# Main source
target_sources(app PRIVATE src/main.c)

# Conditionnal build...
target_sources_ifdef(CONFIG_GPIOHANDLE app PRIVATE src/peripherals/gpio.c)