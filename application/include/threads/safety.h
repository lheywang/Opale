/** ================================================================
 * @file    application/src/threads/safety/safety.h
 *
 * @brief   This file declare the safety subsystem, who regularly
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

#ifndef _DEF_THREADS_SAFETY
#define _DEF_THREADS_SAFETY

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */
#include <zephyr/kernel.h>

/* -----------------------------------------------------------------
 * STRUCTS
 * -----------------------------------------------------------------
 */

struct safety_p1
{
    // Inputs FIFOS
    struct k_fifo &barom_data;
    struct k_fifo &adc_data;
    struct k_fifo &imu_data;
    struct k_fifo &gps_data;
    struct k_fifo &accel_data;

    // Threads handler
    struct k_thread &controller;
    struct k_thread &logger;
    struct k_thread &measures;
};

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_safety(void *p1, void *p2, void *p3);

#endif /* _DEF_THREADS_SAFETY */