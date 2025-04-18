/** ================================================================
 * @file    application/src/main.cpp
 *
 * @brief   This file is the entry point for the application core.
 *          It handle the initialization of all the tasks, and then
 *          launch child threads to handle for each one they're task.
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Custom headers
#include "config.h"

// Threads
#include "threads/controller.h"
#include "threads/logger.h"
#include "threads/safety.h"
#include "threads/threads.h"
#include "threads/measure.h"

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Main, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * ALLOCATING STACK
 * -----------------------------------------------------------------
 */
K_THREAD_STACK_DEFINE(controller_stack, THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(logger_stack, THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(safety_stack, THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(measure_stack, THREAD_STACKSIZE);

/* -----------------------------------------------------------------
 * MAIN LOOP
 * -----------------------------------------------------------------
 */
int main(void)
{
    /* -----------------------------------------------------------------
    * Creating main event to sync threads
    * -----------------------------------------------------------------
    */
    // Allocating struct
    struct k_event globalStatus;

    // Init
    k_event_init(&globalStatus);

    /* -----------------------------------------------------------------
    * Creating main messages queue
    * -----------------------------------------------------------------
    */
    // Allocating struct and memory space
    struct k_msgq threadStatus;
    char threadStatus_buf[10 * sizeof(ThreadStatus)];
    
    // Init
    k_msgq_init(&threadStatus, threadStatus_buf, sizeof(ThreadStatus), 10);

    /* -----------------------------------------------------------------
    * Creating fifos :
    * -----------------------------------------------------------------
    */
    // Allocating structs
    struct k_fifo barom;
    struct k_fifo adc;
    struct k_fifo imu;
    struct k_fifo gps;
    struct k_fifo gpio;
    struct k_fifo accels;

    // Initializing the fifos
    k_fifo_init(&barom);
    k_fifo_init(&adc);
    k_fifo_init(&imu);
    k_fifo_init(&gps);
    k_fifo_init(&gpio);
    k_fifo_init(&accels);

    /* -----------------------------------------------------------------
    * Creating p1 for threads, to give them access to different elements
    * -----------------------------------------------------------------
    */

    struct safety_p1 safety_data = {    .barom_data = barom,
                                        .adc_data = adc,
                                        .imu_data = imu,
                                        .gps_data = gps,
                                        .gpio_data = gpio};

    struct logger_p1 logger_data = {    .barom_data = barom,
                                        .adc_data = adc,
                                        .imu_data = imu,
                                        .gps_data = gps};

    struct controller_p1 controller_data = {    .barom_data = barom,
                                                .adc_data = adc,
                                                .imu_data = imu};

    struct measure_p1 measure_data = {  .barom_data = barom,
                                        .adc_data = adc,
                                        .imu_data = imu};


    /* -----------------------------------------------------------------
    * CREATING THREADS
    * -----------------------------------------------------------------
    */
    struct k_thread controller_thread;
    struct k_thread logger_thread;
    struct k_thread safety_thread;
    struct k_thread measure_thread;

    /* -----------------------------------------------------------------
    * LAUNCHING THREADS
    * -----------------------------------------------------------------
    */

    // Safety checker
    k_thread_create(&safety_thread,
                    safety_stack,
                    K_THREAD_STACK_SIZEOF(safety_stack),
                    thread_safety,
                    (void *)&safety_data,
                    (void *)&globalStatus,
                    (void *)&threadStatus,
                    SAFETY_PRIORITY,
                    0,
                    K_NO_WAIT);

    // Logger
    k_thread_create(&logger_thread,
                    logger_stack,
                    K_THREAD_STACK_SIZEOF(logger_stack),
                    thread_logger,
                    (void *)&logger_data,
                    (void *)&globalStatus,
                    (void *)&threadStatus,
                    LOGGER_PRIORITY,
                    0,
                    K_NO_WAIT);

    // Measurements
    k_thread_create(&measure_thread,
                    measure_stack,
                    K_THREAD_STACK_SIZEOF(measure_stack),
                    thread_measure,
                    (void *)&measure_data,
                    (void *)&globalStatus,
                    (void *)&threadStatus,
                    SAFETY_PRIORITY,
                    0,
                    K_NO_WAIT);

    // Controller
    k_thread_create(&controller_thread, 
        controller_stack, 
        K_THREAD_STACK_SIZEOF(controller_stack),
        thread_controller,
        (void *)&controller_data,
        (void *)&globalStatus,
        (void *)&threadStatus,
        CONTROLLER_PRIORITY,
        0,
        K_NO_WAIT);

    /*
    * Do nothing
    */

    for (;;) k_msleep(1000 * 1000 * 1000); // Sleep for 16 minutes, repeatable...

    return 0;

}

/*
 * Todo: 
 * - Add a init controller, to enable launch of the rocket once every process as booted
 * - Define values of bits for the globalEvent status
 * - Define messages
 */
