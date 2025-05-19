/** ================================================================
 * @file    application/src/threads/controller/controller.cpp
 *
 * @brief   This file implement the main controller of the rocket.
 *          This is this file that decide WHERE we go.
 *
 * @date    07-04-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */
// Header
#include "threads/controller.h"
#include "threads/threads.h"

// Internal libs
#include "config.h"
#include "init/init.hpp"
#include "devices/servo.h"

// Zephyr
#include <zephyr/logging/log.h>

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(CONTROLLER, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_controller(void *p1, void *p2, void *p3)
{
    // init phase : Fetching arguments
    __unused struct controller_p1 *IO = (controller_p1 *)p1;
    __unused struct k_event *globalStatus = (k_event *)p2;
    __unused struct k_msgq *threadStatus = (k_msgq *)p3;

    // Running init code :
    int act_deg = -90;

    // Init peripherals
    struct pwm_dt_spec *pwm_wings = initializer::GetAPWM(PWMS::SERVOS);

    for (;;)
    {
        k_usleep(100 * 1000);

        // Set command
        ServoAngles cmd = {.north = (float)act_deg,
                           .south = (float)act_deg,
                           .east = (float)act_deg,
                           .west = (float)act_deg};

        // Set servo position
        servo::SetPosition(pwm_wings, &cmd);

        // Increment position
        act_deg += 1;
        if (act_deg > 90)
        {
            act_deg = -90;
        }
    }
    return;
}