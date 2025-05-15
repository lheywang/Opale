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
    struct controller_p1 *IO = (controller_p1 *)p1;
    struct k_event *globalStatus = (k_event *)p2;
    struct k_msgq *threadStatus = (k_msgq *)p3;

    // Running init code :

    for (;;)
    {
        LOG_INF("Hi from CONTROLLER !");
        k_usleep(1000 * 1000);
    }
    return;
}