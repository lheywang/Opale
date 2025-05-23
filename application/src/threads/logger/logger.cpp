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

// Internal libs
#include "config.h"
#include "init/init.hpp"
#include "threads/threads.h"

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
    /* -----------------------------------------------------------------
     * Initialization
     * -----------------------------------------------------------------
     */
    // init phase : Fetching arguments
    __unused struct logger_p1 *IO = (logger_p1 *)p1;
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

    // Getting SPI devices openned :
    struct spi_dt_spec *eeprom = initializer::GetAnSPI(SPIS::EEPROM);

    // Set ready bit before enterring main loop
    int thread_rval = ThreadStatus::LOGGER_OK;
    k_event_set_masked(globalStatus, 0xFFFFFFFF, STATUS_LOGGER);
    k_msgq_put(threadStatus, &thread_rval, K_MSEC(50));

    /* -----------------------------------------------------------------
     * Loop
     * -----------------------------------------------------------------
     */

    for (;;)
    {
        // LOG_INF("Hi from LOGGER !");
        k_usleep(1000 * 1000);
    }
    return;
}