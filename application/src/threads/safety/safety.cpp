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
//Header
#include "threads/safety.h"

// Internal libs
#include "config.h"

// Zephyr
#include <zephyr/logging/log.h>

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
    for (;;)
    {
        LOG_INF("Hi from SAFETY !");
        k_usleep(1000 * 1000);
    }
    return;
}