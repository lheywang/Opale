/** ================================================================
 * @file    application/src/threads/controller/controller.h
 *
 * @brief   This file declare the main controller of the rocket.
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

#ifndef _DEF_THREADS_CONTROLLER
#define _DEF_THREADS_CONTROLLER

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */
#include <zephyr/kernel.h>

/* -----------------------------------------------------------------
 * STRUCTS
 * -----------------------------------------------------------------
 */

struct controller_p1
{
    struct k_fifo &barom_data;
    struct k_fifo &adc_data;
    struct k_fifo &imu_data;
    struct k_fifo &accel_data;
};

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_controller(void *p1, void *p2, void *p3);

#endif /* _DEF_THREADS_CONTROLLER */