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
#include <zephyr/device.h>

// Custom headers
#include "init/init.hpp"
#include "config.h"
#include "peripherals/gpio.h"

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
     * First, reseting all of the peripherals at boot / reset
     * -----------------------------------------------------------------
     */
    // Fetch the GPIO and set it as output
    struct gpio_dt_spec *rst = initializer::GetAGPIO(GPIOS::PERIPHERAL_RESET);

    // Trigerring reset procedure
    gpio::SetAsOutput(rst, 1);
    k_msleep(50);
    gpio::Set(rst, 0);
    k_msleep(50);
    gpio::Set(rst, 1);

    // Exiting reset procedure
    initializer::FreeAGPIO(GPIOS::PERIPHERAL_RESET, rst);

    /* -----------------------------------------------------------------
     * Creating main event to sync threads
     * -----------------------------------------------------------------
     */
    // Allocating struct
    struct k_event globalStatus;

    // Init
    k_event_init(&globalStatus);
    k_event_set(&globalStatus, 0x00000000);

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
    // Barometer
    struct k_fifo barom1;
    struct k_fifo barom2;
    struct k_fifo barom3;
    // ADC
    struct k_fifo adc1;
    struct k_fifo adc2;
    struct k_fifo adc3;
    // IMU
    struct k_fifo imu1;
    struct k_fifo imu2;
    struct k_fifo imu3;
    // GPS
    struct k_fifo gps1;
    struct k_fifo gps2;
    // Accels
    struct k_fifo accels1;
    struct k_fifo accels2;

    // Initializing the fifos
    // Barometer
    k_fifo_init(&barom1);
    k_fifo_init(&barom2);
    k_fifo_init(&barom3);
    // ADC
    k_fifo_init(&adc1);
    k_fifo_init(&adc2);
    k_fifo_init(&adc3);
    // IMU
    k_fifo_init(&imu1);
    k_fifo_init(&imu2);
    k_fifo_init(&imu3);
    // GPS
    k_fifo_init(&gps1);
    k_fifo_init(&gps2);
    // Accels
    k_fifo_init(&accels1);
    k_fifo_init(&accels2);

    /* -----------------------------------------------------------------
     * CREATING THREADS
     * -----------------------------------------------------------------
     */
    struct k_thread controller_thread;
    struct k_thread logger_thread;
    struct k_thread safety_thread;
    struct k_thread measure_thread;

    /* -----------------------------------------------------------------
     * Creating p1 for threads, to give them access to different elements
     * -----------------------------------------------------------------
     */

    struct safety_p1 safety_data = {.barom_data = barom1,
                                    .adc_data = adc1,
                                    .imu_data = imu1,
                                    .gps_data = gps1,
                                    .accel_data = accels1,
                                    .controller = controller_thread,
                                    .logger = logger_thread,
                                    .measures = measure_thread};

    struct logger_p1 logger_data = {.barom_data = barom3,
                                    .adc_data = adc3,
                                    .imu_data = imu3,
                                    .gps_data = gps2};

    struct controller_p1 controller_data = {.barom_data = barom2,
                                            .adc_data = adc2,
                                            .imu_data = imu2,
                                            .accel_data = accels2};

    struct measure_p1 measure_data = {.barom_data1 = barom1,
                                      .barom_data2 = barom2,
                                      .barom_data3 = barom3,
                                      .adc_data1 = adc1,
                                      .adc_data2 = adc2,
                                      .adc_data3 = adc3,
                                      .imu_data1 = imu1,
                                      .imu_data2 = imu2,
                                      .imu_data3 = imu3,
                                      .gps_data1 = gps1,
                                      .gps_data2 = gps2,
                                      .accels_data1 = accels1,
                                      .accels_data2 = accels2};

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
     * Do nothing (but remain alive !)
     *
     * If we exit the task, memory if freed and we then loose FIFOS.
     * This trigger a secure fault errors on the CPU.
     */

    for (;;)
        k_msleep(1000 * 1000 * 1000); // Sleep for 16 minutes, repeatable...

    /*
     * We shall never get here...
     */
    return 0;
}
