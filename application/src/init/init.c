/** ================================================================
 * @file    application/src/init/init.c
 *
 * @brief   Init.c define the function that check if the peripherals
 *          are ready to be used, or not.
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
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

// Custom libs
#include "init.h"
#include "../config.h"

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Initializer, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * PRIVATES FUNCTIONS (For a single peripheral instance)
 * -----------------------------------------------------------------
 */

int CheckAGPIO(const struct gpio_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!gpio_is_ready_dt(&Target->port))
        {
            if (k < 2)
            {
                LOG_WRN("%s:%d is not ready for now. Retrying in 5 ms...", Target->port->name, Target->pin);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s:%d is not working properly.", Target->port->name, Target->pin);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s:%d is working correctly !", Target->port->name, Target->pin);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAPWM(const struct pwm_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!pwm_is_ready_dt(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s:%d is not ready for now. Retrying in 5 ms...", Target->dev->name, Target->channel);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s:%d is not working properly.", Target->dev->name, Target->channel);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s:%d is working correctly !", Target->dev->name, Target->channel);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAnUART(const struct device *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!device_is_ready(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s is not ready for now. Retrying in 5 ms...", Target->name);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s is not working properly.", Target->name);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s is working correctly !", Target->name);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAnI2C(const struct i2c_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!device_is_ready(Target->bus))
        {
            if (k < 2)
            {
                LOG_WRN("%s@0x%xRW is not ready for now. Retrying in 5 ms...", Target->bus->name, Target->addr);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s@0x%xRW is not working properly.", Target->bus->name, Target->addr);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s@0x%xRW is working correctly !", Target->bus->name, Target->addr);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAnSPI(const struct spi_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!spi_is_ready_dt(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s@%d is not ready for now. Retrying in 5 ms...",
                        Target->bus->name,
                        Target->config.cs.gpio.pin);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s@%d is not working properly.",
                        Target->bus->name,
                        Target->config.cs.gpio.pin);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s@%d is working correctly !",
                    Target->bus->name,
                    Target->config.cs.gpio.pin);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}
/* -----------------------------------------------------------------
 * FUNCTIONS TO CHECK IF THE PERIPHERAL IS OK
 * -----------------------------------------------------------------
 */

int INIT_CheckGPIO()
{

    int ErrCounter = 0;

    for (uint8_t k = 0; k < 4; k++)
        ErrCounter += CheckAGPIO(&engines[k]);

    for (uint8_t k = 0; k < 3; k++)
        ErrCounter += CheckAGPIO(&inputs[k]);

    ErrCounter += CheckAGPIO(&peripheral_reset);
    ErrCounter += CheckAGPIO(&rocket_latch);
    ErrCounter += CheckAGPIO(&imu_boot);
    ErrCounter += CheckAGPIO(&imu_status);
    ErrCounter += CheckAGPIO(&rocket_mode);
    ErrCounter += CheckAGPIO(&imu_int);
    ErrCounter += CheckAGPIO(&gps_int);
    ErrCounter += CheckAGPIO(&accel1_int);
    ErrCounter += CheckAGPIO(&accel2_int);

    return -ErrCounter;
}

int INIT_CheckPWM()
{

    int ErrCounter = 0;

    for (uint8_t i = 0; i < PWM_SERVO_LEN; i++)
        ErrCounter += CheckAPWM(&pwm_wings[i]);

    for (uint8_t i = 0; i < PWM_RGB_LEN; i++)
        ErrCounter += CheckAPWM(&pwm_rgb[i]);

    ErrCounter += CheckAPWM(&pwm_buzzer);
    ErrCounter += CheckAPWM(&pwm_parachute);
    return -ErrCounter;
}

int INIT_CheckUART()
{

    int ErrCounter = 0;

    ErrCounter += CheckAnUART(uart_imu);
    ErrCounter += CheckAnUART(uart_gps);
    return -ErrCounter;
}

int INIT_CheckI2C()
{

    int ErrCounter = 0;

    ErrCounter += CheckAnI2C(&i2c_barometer);
    ErrCounter += CheckAnI2C(&i2c_expander);

    for (uint8_t i = 0; i < ACCEL_NB; i++)
        ErrCounter += CheckAnI2C(&i2c_accels[i]);
    return -ErrCounter;
}

int INIT_CheckSPI()
{

    int ErrCounter = 0;
    ErrCounter += CheckAnSPI(&spi_eeproms);
    return -ErrCounter;
}

int INIT_CheckUSB()
{
    if (usb_enable(NULL))
        return -1;
    return 0;
}