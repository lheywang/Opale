/** ================================================================
 * @file    application/src/main.c
 *
 * @brief   This file is the entry point for the application core.
 *          It handle the initialization of all the tasks, and then
 *          launch child threads to handle for each one they're task.
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

// Custom headers
#include "init/init.h"
#include "config.h"

#include "devices/servo.h"
#include "devices/rgb.h"
#include "devices/gpio.h"
#include "devices/eeprom.h"
#include "devices/saadc.h"

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Main, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * INCLUDING VARIABLES
 * -----------------------------------------------------------------
 */

// Servo PWM
extern const struct pwm_dt_spec pwm0_servo0;
extern const struct pwm_dt_spec pwm0_servo1;
extern const struct pwm_dt_spec pwm0_servo2;

// Ouputs
extern const struct gpio_dt_spec peripheral_reset;
extern const struct gpio_dt_spec rocket_latch;
extern const struct gpio_dt_spec engines[4];
// Inputs
extern const struct gpio_dt_spec imu_boot;
extern const struct gpio_dt_spec imu_status;
extern const struct gpio_dt_spec rocket_mode;
// Interruptibles inputs
extern const struct gpio_dt_spec imu_int;
extern const struct gpio_dt_spec gps_int;
extern const struct gpio_dt_spec accel1_int;
extern const struct gpio_dt_spec accel2_int;
extern const struct gpio_dt_spec inputs[3];

// UARTS
extern const struct device *uart_imu;
extern const struct device *uart_gps;

// Servo engines (wings) commands
extern const struct pwm_dt_spec pwm_wings[PWM_SERVO_LEN];

extern const struct pwm_dt_spec pwm_rgb[PWM_RGB_LEN];

extern const struct pwm_dt_spec pwm_buzzer;
extern const struct pwm_dt_spec pwm_parachute;

// SPI devices
extern const struct spi_dt_spec spi_eeproms;

// I2C devices
extern const struct i2c_dt_spec i2c_barometer;
extern const struct i2c_dt_spec i2c_expander;
extern const struct i2c_dt_spec i2c_accels[ACCEL_NB];

// Timers
extern nrfx_timer_t saadc_timer;

// ADC outputs
extern uint16_t saadc_buffer[2][SAADC_BUFFER_SIZE];
extern uint32_t saadc_buffer_index;

/* -----------------------------------------------------------------
 * MAIN LOOP
 * -----------------------------------------------------------------
 */
int main(void)
{
    /* -----------------------------------------------------------------
    * PERIPHERALS INITS
    * -----------------------------------------------------------------
    */
    int err = 0;
    err -= INIT_CheckGPIO();
    err -= INIT_CheckPWM();
    err -= INIT_CheckUART();
    err -= INIT_CheckI2C();
    err -= INIT_CheckSPI();
    err -= INIT_CheckUSB();

    if (err != 0)
    {
        LOG_ERR("Some peripherals aren't fine. Aborting...");
        return 0;
    }

    int ret = GPIO_SetAsOutput(&peripheral_reset, 0);
    ret -= SAADC_Configure(&saadc_timer);

    if (ret < 0)
        return 0;

    /* -----------------------------------------------------------------
    * INITIALIZING EXTERNAL DEVICES TO KNOWN POSITION
    * -----------------------------------------------------------------
    */

    ServoAngles Command = {.north = 90,
                           .south = 0,
                           .east = -90,
                           .west = 0};

    ret += SERVO_SetPosition(pwm_wings, &Command);

    Color Command2 = {.red = 255,
                      .green = 127,
                      .blue = 0,
                      .alpha = 100};

    ret += RGB_SetColor(pwm_rgb, &Command2);

    /* -----------------------------------------------------------------
    * MAIN LOOP
    * -----------------------------------------------------------------
    */

    while (1)
    {
        LOG_INF("New cycle started !");
        k_yield();

        // Command 1
        ServoAngles Command = { .north = 90,
                                .south = 45,
                                .east = -17.5,
                                .west = -50};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 2
        ServoAngles Command = { .north = 90,
                                .south = 45,
                                .east = -12.5,
                                .west = -10};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 3
        ServoAngles Command = { .north = -90,
                                .south = -45,
                                .east = -7.5,
                                .west = -30};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 4
        ServoAngles Command = { .north = -90,
                                .south = -45,
                                .east = -2.5,
                                .west = 10};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 5
        ServoAngles Command = { .north = 90,
                                .south = 45,
                                .east = 2.5,
                                .west = -10};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 6
        ServoAngles Command = { .north = 90,
                                .south = 45,
                                .east = -7.5,
                                .west = 30};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 7
        ServoAngles Command = { .north = -90,
                                .south = -45,
                                .east = -12.5,
                                .west = 10};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);

        // Command 8
        ServoAngles Command = { .north = -90,
                                .south = -45,
                                .east = -17.5,
                                .west = 50};
        ret += SERVO_SetPosition(pwm_wings, &Command);
        if (ret != 0)
            return 0;
        k_msleep(2500);
    }
}

/*
 * This sample make moves the engine position following different commands :
 * - north engine : alternate betweeen 90 and -90 every 5 seconds
 * - south engine : alternate between 46 and -45 every 5 seconds
 * - east engine : discrete ramp, +5 degree every 2.5 second. (with a final step of -80 degree to restart).
 * - west engine : random response : -50, -10, -30, 10, -10, 30, 10, 50 degrees to measure random response.
 * 
 * - ADC shall print in console measurements per channel every 2.5 seconds to match the response.
 * 
 *  */
