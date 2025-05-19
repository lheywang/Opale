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

// Internal libs
#include "config.h"
#include "init/init.hpp"
#include "threads/threads.h"

// Devices
#include "devices/rgb.h"
#include "devices/rocket.h"
#include "peripherals/gpio.h"
// Drivers
#include "drivers/bno055.h"
#include "drivers/MS5611.h"

// Zephyr
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>

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
    /* -----------------------------------------------------------------
     * Initialization (1st part)
     * -----------------------------------------------------------------
     */
    __unused struct safety_p1 *IO = (safety_p1 *)p1;
    struct k_event *globalStatus = (k_event *)p2;
    __unused struct k_msgq *threadStatus = (k_msgq *)p3;

    // Fetch peripherals structures
    struct pwm_dt_spec *pwm_rgb = initializer::GetAPWM(PWMS::RGB);
    struct pwm_dt_spec *pwm_buzzer = initializer::GetAPWM(PWMS::BUZZER);
    struct gpio_dt_spec *rocket_mode = initializer::GetAGPIO(GPIOS::MODE);
    struct gpio_dt_spec *rocket_latch = initializer::GetAGPIO(GPIOS::LATCH);

    // Set led to orange to indicate the user that the system is booting
    Color Command = {.red = 0xFF,
                     .green = 0x99,
                     .blue = 0x1C,
                     .alpha = 100};
    rgb::SetColor(pwm_rgb, &Command);

    // Get the rocket operation mode
    OperationMode mode;
    rocket::GetOperationMode(rocket_latch, rocket_mode, &mode);

    // Set ready bit before enterring main loop
    int thread_rval = ThreadStatus::SAFETY_OK;
    k_event_set_masked(globalStatus, 0xFFFFFFFF, STATUS_SAFETY);
    k_msgq_put(threadStatus, &thread_rval, K_MSEC(50));

    // Wait for the order of boot to be valid :
    bool wait = true;
    while (wait == true)
    {
        uint32_t val = k_event_test(globalStatus, STATUS_MEASURES);
        wait = (val != 0) ? false : true;
        k_msleep(20);
    }

    /* -----------------------------------------------------------------
     * Initialization (2nd part)
     * -----------------------------------------------------------------
     */

    // Computing the overall mask
    uint32_t mask_threads = STATUS_SAFETY |
                            STATUS_LOGGER |
                            STATUS_MEASURES |
                            STATUS_CONTROLLER;
    uint32_t mask_peripherals = STATUS_ADC |
                                STATUS_GPS |
                                STATUS_ACCELS |
                                STATUS_BAROM |
                                STATUS_GPIOS |
                                STATUS_SERVOS |
                                STATUS_SPI;
    uint32_t mask = mask_threads | mask_peripherals;

    // Always OK ??

    // Checking that all of the
    uint32_t val = k_event_test(globalStatus, mask);
    if (val == 0)
    {
        // Something went wrong...
        LOG_ERR("A peripheral wasn't operational. Blocked launch procedure. Status register : %08x", val);

        // Set led to red to indicate the user that the system is not ready and won't be.
        Color Command = {.red = 0xCD,
                         .green = 0x1C,
                         .blue = 0x18,
                         .alpha = 100};
        rgb::SetColor(pwm_rgb, &Command);
    }
    else
    {
        // Nothing to be warned
        LOG_INF("Everything is correct. Continuing procedure...");

        // Set the corresponding bits to the register value
        k_event_set_masked(globalStatus, 0xFFFFFFFF, STATUS_AUTH_CONTROL);
        k_event_set_masked(globalStatus, 0xFFFFFFFF, STATUS_AUTH_LAUNCH);

        // Set led to green to indicate the user that the system is ready
        Color Command = {.red = 0x50,
                         .green = 0xC8,
                         .blue = 0x78,
                         .alpha = 100};
        rgb::SetColor(pwm_rgb, &Command);
    }

    /* -----------------------------------------------------------------
     * Loop
     * -----------------------------------------------------------------
     */

    pwm_set_dt(pwm_buzzer, PWM_KHZ(0.5), PWM_KHZ(1));

    for (;;)
    {
        // Wait 50 ms : Full cycle is ~10 s
        k_usleep(50 * 1000);
    }
    return;
}