/** ================================================================
 * @file    application/src/devices/rgb.cpp
 *
 * @brief   rgb.cpp implement some low level OS call to configure the
 *          color of an RGB led.
 *
 * @date    20-02-2025
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
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

// Local libs
#include "devices/rgb.h"
#include "config.h"
#include "init/init.hpp"

// STD
#include <math.h>

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(RGB, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND A SERVO
 * -----------------------------------------------------------------
 */

int rgb::SetColor(const struct pwm_dt_spec Target[PWM_RGB_LEN],
                  Color const *Command)
{

    // Checking the parameters
    // MIN =< Pos <= MAX
    if (Command->alpha > 100)
    {
        LOG_ERR("Incorrect command passed on the alpha value. (MAX = 100)");
        return -1;
    }

    // Computing values$
    // pulse[0] = red, pulses[1] = green, pulses[2] = blue
    double pulses[3] = {0};

    pulses[0] = Command->red;
    pulses[1] = Command->green;
    pulses[2] = Command->blue;

    // Apply the alpha value
    pulses[0] *= (Command->alpha / 100.0);
    pulses[1] *= (Command->alpha / 100.0);
    pulses[2] *= (Command->alpha / 100.0);

    // Compute the pulses values
    pulses[0] *= (PWM_RGB_PERIOD / 255.0);
    pulses[1] *= (PWM_RGB_PERIOD / 255.0);
    pulses[2] *= (PWM_RGB_PERIOD / 255.0);

    // Configure the PWM engines
    int err = 0;
    for (uint8_t k = 0; k < 3; k++)
    {
        err += pwm_set_dt(&Target[k], PWM_RGB_PERIOD, (int)round(pulses[k]));
    }

    // End log, and return
    if (err != 0)
    {
        LOG_ERR("Failed to co<nfigure the PWM duty cycle for the RGB leds : %d", err);
        return -2;
    }
    return 0;
}