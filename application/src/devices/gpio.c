/** ================================================================
 * @file    application/src/devices/gpio.c
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
#include "gpio.h"

/* -----------------------------------------------------------------
* FUNCTIONS TO COMMAND A SERVO
* -----------------------------------------------------------------
*/


int GPIOSetAsInput(const struct gpio_dt_spec *Target){
    return gpio_pin_configure_dt(Target, GPIO_INPUT);
}


int GPIOSetAsOutput(const struct gpio_dt_spec *Target, const uint8_t State){
    if (State){
        return gpio_pin_configure_dt(Target, GPIO_OUTPUT_ACTIVE);
    }
    return gpio_pin_configure_dt(Target, GPIO_OUTPUT_INACTIVE);
}


int GPIOSetAsHighZ(const struct gpio_dt_spec *Target){
    return gpio_pin_configure_dt(Target, GPIO_DISCONNECTED);
}


int GPIOToggle(const struct gpio_dt_spec *Target){
    return gpio_pin_toggle_dt(Target);
}


int GPIOSet(const struct gpio_dt_spec *Target, const int Value){
    return gpio_pin_set_dt(Target, Value);
}


int GPIORead(const struct gpio_dt_spec *Target, int *const Value){
    *Value = gpio_pin_get_dt(Target);
    return 0;
}