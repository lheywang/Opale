/** ================================================================
 * @file    application/src/threads/logger/logger.h
 *
 * @brief   This file declare the logger system, that logs data
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

#ifndef _DEF_THREADS_LOGGER
#define _DEF_THREADS_LOGGER

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */
#include <zephyr/kernel.h>


/* -----------------------------------------------------------------
 * STRUCTS
 * -----------------------------------------------------------------
 */

struct logger_p1 {
    struct k_fifo &barom_data;
    struct k_fifo &adc_data;
    struct k_fifo &imu_data;
    struct k_fifo &gps_data;
};

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_logger(void *p1, void *p2, void *p3);

#endif /* _DEF_THREADS_LOGGER */