/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/config.h
 *
 * @brief   Define the standard GPIO used for different usages, 
 *          and some standard configs for the Opale project.
 *
 * @details All of the GPIOs on the nRF5340 are mappable, which 
 *          mean we can can assign any function to any output 
 *          (within some limits, relatives to analog or RF 
 *          functions...).
 *          The rest of this file define pins for the Application 
 *          core, which handle most of the computation about 
 *          flight control.
 *          All of theses pins are identfied by two values : 
 *          Pin, and port.
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 * @date    15/02/2025
 *
 * @version 1.0.0
 *
 * ---------------------------------------------------------------
 */
#pragma once

// ----------------------------
// PWM
// ----------------------------
// -- Wings command :
// Wing 1 : P1.14
#define PIN_PWM1_COMMAND_OUT1   14
#define PORT_PWM1_COMMAND_OUT1  1
// Wing 2 : P1.12
#define PIN_PWM1_COMMAND_OUT2   12
#define PORT_PWM1_COMMAND_OUT2  1
// Wing 3 : P1.0
#define PIN_PWM1_COMMAND_OUT3   0
#define PORT_PWM1_COMMAND_OUT3  1
// Wing 4 : P0.2
#define PIN_PWM1_COMMAND_OUT4   2
#define PORT_PWM1_COMMAND_OUT4  0

// -- Parachute
// Parachute : P1.1
#define PIN_PMW2_COMMAND_OUT1   1
#define PORT_PWM2_COMMAND_OUT1  1

// -- Buzzer
// Buzzer : P0.17
#define PIN_PWM3_COMMAND_OUT1   17
#define PORT_PWM3_COMMAND_OUT1  0

// -- RGB LED
// RED P0.30
#define PIN_PMW4_COMMAND_RED    30
#define PORT_PWM4_COMMAND_RED   0
// GREEN P1.10
#define PIN_PMW4_COMMAND_GREEN  10
#define PORT_PWM4_COMMAND_GREEN 1
// BLUE P0.29
#define PIN_PMW4_COMMAND_BLUE   30
#define PORT_PWM4_COMMAND_BLUE  0

// ----------------------------
// SPI (Master)
// ----------------------------
// -- EEPROM communication
// MOSI P0.19
#define PIN_SPIM_MOSI           19
#define PORT_SPIM_MOSI          0
// MISO P1.04
#define PIN_SPIM_MISO           4
#define PORT_SPIM_MISO          1
// SCLK P0.21
#define PIN_SPIM_SCLK           21
#define PORT_SPIM_SCLK          0
// CS1 P1.08
#define PIN_SPIM_CS1            08
#define PORT_SPIM_CS1           1
// CS2 P1.05
#define PIN_SPIM_CS2            05
#define PORT_SPIM_CS2           1
// CS3 P1.06
#define PIN_SPIM_CS3            06
#define PORT_SPIM_CS3           1

// ----------------------------
// I2C (Master)
// ----------------------------
// -- Sensors communication
// SDA P0.13
#define PIN_TWIS_SDA            13
#define PORT_TWIS_SDA           0
// SCL P0.12
#define PIN_TWIS_SCL            12
#define PORT_TWIS_SCL           0

// ----------------------------
// UARTE 
// ----------------------------
// -- BNO055 (Already inverted against schematic)
// RX1 P1.02
#define PIN_UARTE_BNO055_TX     2
#define PORT_UARTE_BNO055_TX    1
// TX1 P0.09
#define PIN_UARTE_BNO055_RX     9
#define PORT_UARTE_BNO055_RX    0

// -- LIV3R (Already inverted against schematic)
// RX2 P0.31
#define PIN_UARTE_LIV3R_TX      31
#define PORT_UARTE_LIV3R_TX     0
// TX2 P1.11
#define PIN_UARTE_LIV3R_RX      11
#define PORT_UARTE_LIV3R_RX     1

// ----------------------------
// ANALOG_INPUTS
// ----------------------------
// -- Power sensing 
// Vbat P0.25
#define PIN_SAADC_VBAT          25
#define PORT_SAADC_VBAT         0
// Vbat aux (pyro) P0.26
#define PIN_SAADC_VBATPYRO      26
#define PORT_SAADC_VBATPYRO     0
// Vbus P0.07
#define PIN_SAADC_VBUS          7
#define PORT_SAADC              0
// 3V3 P0.06
#define PIN_SAADC_3V3           6
#define PORT_SAADC_3V3          0

// -- Wings feedback
// Wing 1 P0.28
#define PIN_SAADC_WING1POS      28
#define PORT_SAADC_WING1POS     0
// Wing 2 P0.27
#define PIN_SAADC_WING2POS      27
#define PORT_SAADC_WING2POS     0
// Wing 3 P0.05
#define PIN_SAADC_WING3POS      5
#define PORT_SAADC_WING3POS     0
// Wing 4 P0.04
#define PIN_SAADC_WING4POS      4
#define PORT_SAADC_WING4POS     0

// ----------------------------
// Counter inputs
// ----------------------------
// -- 1s CLK (GPS)
// CLKOUT P1.15
#define PIN_TIMER_INPUT_COUNT1  15
#define PORT_TIMER_INPUT_COUNT1 1

// ----------------------------
// GPIOS (Interrupts)
// ----------------------------
// -- Sensors interrupts
// GPS P1.13
#define PIN_GPIO_IN_INTGPS      13
#define PORT_GPIO_IN_INTGPS     1
// IMU P0.10
#define PIN_GPIO_IN_INTIMU      10
#define PORT_GPIO_IN_INTIMU     0
// ACCEL P0.23
#define PIN_GPIO_IN_INTACCEL1   23
#define PORT_GPIO_IN_INTACCEL1  0
// ACCEL P0.20
#define PIN_GPIO_IN_INTACCEL2   20
#define PORT_GPIO_IN_INTACCEL2  0

// ----------------------------
// GPIOS (Standard)
// ----------------------------
// -- Inputs
// IMU9_STATUS P0.15
#define PIN_GPIO_IN_IMU9STATUS  15
#define PORT_GPIO_IN_IMU9STATUS 0
// IMU9_BOOT P0.03
#define PIN_GPIO_IN_IMU9BOOT    3
#define PORT_GPIO_IN_IMU9BOOT   0
// Unsafe Input 1 P0.07
#define PIN_GPIO_IN_GPIN1       7
#define PORT_GPIO_IN_GPIN1      0
// Unsafe Input 2 P0.07
#define PIN_GPIO_IN_GPIN2       9
#define PORT_GPIO_IN_GPIN2      0
// Unsafe Input 3 P0.24
#define PIN_GPIO_IN_GPIN3       24
#define PORT_GPIO_IN_GPIN3      0
//Rocket flight mode P0.11
#define PIN_GPIO_IN_ROCKETMODE  11
#define PORT_GPIO_IN_ROCKETMODE 0

// -- Outputs
// Peripheral reset P1.03
#define PIN_GPIO_OUT_SENS_RST   3
#define PORT_GPIO_OUT_SENS_RST  1
// Latch rocket mode P0.08
#define PIN_GPIO_OUT_LATCHMODE  8
#define PORT_GPIO_OUT_LATCHMODE 0

