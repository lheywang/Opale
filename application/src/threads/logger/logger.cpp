/** ================================================================
 * @file    application/src/threads/logger/logger.cpp
 *
 * @brief   This file implement the logger system, that logs data
 *          into the eeprom, as well as outputing it on the logs.
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
#include "threads/logger.h"
#include "threads/threads.h"

// Internal libs
#include "config.h"
#include "init/init.hpp"

// Zephyr
#include <zephyr/logging/log.h>

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(LOGGER, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_logger(void *p1, void *p2, void *p3)
{
    // init phase : Fetching arguments
    __unused struct logger_p1 *IO = (logger_p1 *)p1;
    __unused struct k_event *globalStatus = (k_event *)p2;
    __unused struct k_msgq *threadStatus = (k_msgq *)p3;

    // Running init code :

    for (;;)
    {
        // LOG_INF("Hi from LOGGER !");
        k_usleep(1000 * 1000);
    }
    return;
}