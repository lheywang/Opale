/** ================================================================
 * @file    application/src/threads/safety/safety.cpp
 *
 * @brief   This file implement the safety subsystem, who regularly
 *          check that everything is fine !
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
#include "threads/safety.h"
#include "threads/threads.h"

// Internal libs
#include "config.h"
#include "init/init.hpp"
#include "devices/rgb.h"
#include "drivers/bno055.h"
#include "drivers/MS5611.h"
#include "devices/rocket.h"

// Zephyr
#include <zephyr/logging/log.h>

// STD
#include "math.h"

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(SAFETY, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_safety(void *p1, void *p2, void *p3)
{
    // init phase : Fetching arguments
    __unused struct safety_p1 *IO = (safety_p1 *)p1;
    __unused struct k_event *globalStatus = (k_event *)p2;
    __unused struct k_msgq *threadStatus = (k_msgq *)p3;

    // Running init code :

    // Inititalize peripherals
    struct pwm_dt_spec *pwm_rgb = initializer::GetAPWM(PWMS::RGB);
    struct gpio_dt_spec *rocket_mode = initializer::GetAGPIO(GPIOS::MODE);
    struct gpio_dt_spec *rocket_status = initializer::GetAGPIO(GPIOS::LATCH);
    MS5611 barometer = MS5611();

    // Define some parameters and constants
    double t = 0.0;
    double inc = 0.031415976; // ~ Pi / 100
    double p0 = 0.0;
    double p120 = 2.0943951024;
    double p240 = 4.1887902048;

    // Get the rocket operation mode
    OperationMode mode;
    int tmp = rocket::GetOperationMode(rocket_mode, rocket_status, &mode);
    LOG_INF("Retval GPIO %d", tmp);

    switch (mode)
    {
    case OperationMode::LAUNCH:
        LOG_INF("Rocket is mode LAUNCH");
    case OperationMode::DEBUG:
        LOG_INF("Rocket is mode DEBUG");
    }

    for (;;)
    {
        // Wait 50 ms : Full cycle is ~10 s
        k_usleep(50 * 1000);

        // Compute the color (using cosines and phases)
        Color Command = {.red = (uint8_t)round((cos((t + p0)) + 1) * 127),
                         .green = (uint8_t)round((cos((t + p120)) + 1) * 127),
                         .blue = (uint8_t)round((cos((t + p240)) + 1) * 127),
                         .alpha = 50};

        // Apply the color
        rgb::SetColor(pwm_rgb, &Command);

        // Increment time
        t += inc;
    }
    return;
}