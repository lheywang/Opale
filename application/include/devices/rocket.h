/** ================================================================
 * @file    application/src/devices/rocket.h
 *
 * @brief   servo.h define some higher level interractions with the
 *          servos, for example configuring their positions, or
 *          reading back the actual position.
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

// Include once
#ifndef DEF_ROCKET
#define DEF_ROCKET

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */
// Zephyr
#include <zephyr/drivers/gpio.h>

// Libs
#include "init/init.hpp"
#include "peripherals/gpio.h"

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */
typedef enum
{
    DEBUG,
    LAUNCH
} OperationMode;

/* -----------------------------------------------------------------
 * FUNCTIONS TO FETCH THE MODE
 * -----------------------------------------------------------------
 */

namespace rocket
{
    /**
     * @brief   Return the rocket operation mode, between LAUNCH or DEBUG.
     *          Theses are used to ensure a safety while operating the rocket.
     *
     * @param   Latch   The GPIO used to latch the D-Flip-Flop gate.
     * @param   Status  The GPIO used to read the D-Flip-Flop status.
     * @param   Mode    The return value.
     *
     * @return  0   Operation complete.
     * @return -1   Error while calling kernel GPIO procedures.
     */
    int GetOperationMode(const struct gpio_dt_spec *Latch,
                         const struct gpio_dt_spec *Status,
                         OperationMode *const Mode);
}

#endif /* DEF_ROCKET*/
