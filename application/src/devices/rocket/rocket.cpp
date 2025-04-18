/** ================================================================
 * @file    application/src/devices/rocket.cpp
 *
 * @brief   rocket.cpp implement low level rocket functions, mainly
 *          for the launch authorization.
 *
 * @date    23-02-2025
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
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// Local libs
#include "devices/rocket.h"
#include "peripherals/gpio.h"
#include "config.h"

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Rocket, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * FUNCTIONS TO FETCH THE MODE
 * -----------------------------------------------------------------
 */

int rocket::GetOperationMode(const struct gpio_dt_spec *Latch,
                            const struct gpio_dt_spec *Status,
                            OperationMode *const Mode)
{

    // First, trigger the GPIO latching process
    if (!gpio::Set(Latch, 1))
        return 1;

    k_msleep(5);
    if (!gpio::Set(Latch, 0))
        return -1;

    // Then, read back the value
    k_msleep(5);
    int tmp = 0;
    gpio::Read(Status, &tmp);

    // Copy the data back.
    *Mode = OperationMode(tmp);
    return 0;
}
