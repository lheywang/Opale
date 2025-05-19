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
#include "threads/threads.h"

// Internal libs
#include "config.h"
#include "init/init.hpp"

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
    // init phase : Fetching arguments
    __unused struct measure_p1 *IO = (measure_p1 *)p1;
    __unused struct k_event *globalStatus = (k_event *)p2;
    __unused struct k_msgq *threadStatus = (k_msgq *)p3;

    // Running init code :
    struct fifo_data tmp;
    tmp.value = 0;

    for (;;)
    {
        // LOG_INF("Hi from MEASURE ! Count = %d", tmp.value);

        // k_fifo_put(&IO->barom_data1, &tmp);

        k_usleep(1000 * 1000);
        tmp.value++;
    }
    return;
}