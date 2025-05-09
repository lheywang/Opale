/** ================================================================
 * @file    application/src/peripherals/gpio/gpio.cpp
 *
 * @brief   rgb.c implement some higher level functions to control
 *          GPIO
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
#include <zephyr/drivers/gpio.h>

// Local libs
#include "peripherals/gpio.h"

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND A SERVO
 * -----------------------------------------------------------------
 */

int gpio::SetAsInput(const struct gpio_dt_spec *Target)
{
    return gpio_pin_configure_dt(Target, GPIO_INPUT);
}

int gpio::SetAsOutput(const struct gpio_dt_spec *Target, const uint8_t State)
{
    if (State)
    {
        return gpio_pin_configure_dt(Target, GPIO_OUTPUT_ACTIVE);
    }
    return gpio_pin_configure_dt(Target, GPIO_OUTPUT_INACTIVE);
}

int gpio::SetAsHighZ(const struct gpio_dt_spec *Target)
{
    return gpio_pin_configure_dt(Target, GPIO_DISCONNECTED);
}

int gpio::Toggle(const struct gpio_dt_spec *Target)
{
    return gpio_pin_toggle_dt(Target);
}

int gpio::Set(const struct gpio_dt_spec *Target, const int Value)
{
    return gpio_pin_set_dt(Target, Value);
}

int gpio::Read(const struct gpio_dt_spec *Target, int *const Value)
{
    *Value = gpio_pin_get_dt(Target);
    return 0;
}