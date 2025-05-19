/** ================================================================
 * @file    application/src/threads/measure/measure.cpp
 *
 * @brief   This file implement the measurement engine of the rocket.
 *
 * @date    18-04-2025
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
#include "threads/measure.h"

// Internal libs
#include "config.h"
#include "init/init.hpp"
#include "threads/threads.h"

// Include drivers
#include "drivers/teseo.h"
#include "drivers/MS5611.h"
#include "drivers/bno055.h"
#include "drivers/iis2dlpc_reg.h"
#include "drivers/mcp23008.h"

// Zephyr
#include <zephyr/logging/log.h>

// STD
#include <math.h>

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(MEASURER, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_measure(void *p1, void *p2, void *p3)
{
    /* -----------------------------------------------------------------
     * Initialization
     * -----------------------------------------------------------------
     */

    // init phase : Fetching arguments
    __unused struct measure_p1 *IO = (measure_p1 *)p1;
    struct k_event *globalStatus = (k_event *)p2;
    __unused struct k_msgq *threadStatus = (k_msgq *)p3;

    // Wait for the order of boot to be valid :
    bool wait = true;
    while (wait == true)
    {
        uint32_t val = k_event_test(globalStatus, STATUS_SAFETY);
        wait = (val != 0) ? false : true;
        k_msleep(20);
    }

    // Openning classes
    MS5611 barometer = MS5611();

    // Set ready bit before enterring main loop
    int thread_rval = ThreadStatus::MEASURE_OK;
    k_event_set_masked(globalStatus, 0xFFFFFFFF, STATUS_MEASURES);
    k_msgq_put(threadStatus, &thread_rval, K_MSEC(50));

    /* -----------------------------------------------------------------
     * Loop
     * -----------------------------------------------------------------
     */

    for (;;)
    {
        k_usleep(1000 * 1000);
    }
    return;
}