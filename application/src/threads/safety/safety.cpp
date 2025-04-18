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
LOG_MODULE_REGISTER(SAFETY, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_safety(void *p1, void *p2, void *p3)
{
    // init phase : Fetching arguments
    struct safety_p1 *IO = (safety_p1*)p1;
    struct k_event *globalStatus = (k_event*)p2;
    struct k_msgq *threadStatus = (k_msgq*)p3;

    // Running init code :

    for (;;)
    {
        k_usleep(1000 * 1000);

        struct fifo_data *tmp = (fifo_data*)k_fifo_get(&IO->barom_data, K_FOREVER);

        LOG_INF("Hi from SAFETY ! Value for CNT %d", tmp->value);  
    }
    return;
}