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

// Zephyr
#include <zephyr/logging/log.h>

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
    for (;;)
    {
        LOG_INF("Hi from MEASURE !");
        k_usleep(1000 * 1000);
    }
    return ;
}