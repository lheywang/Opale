/** ================================================================
 * @file    application/src/threads/threads.h
 *
 * @brief   This file declare main settings for the threads used by
 *          our project
 *
 * @date    07-04-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

#ifndef _DEF_THREADS
#define _DEF_THREADS

/* -----------------------------------------------------------------
 * DEFINES
 * -----------------------------------------------------------------
 */
// Stack size for threads
#define THREAD_STACKSIZE    1024

/*
 *  To mention the base of RTOS programming :
 *
 *  "   Threads are assigned an integer value to 
 *  indicate their priority, which can be either 
 *  negative or non-negative. Lower numerical 
 *  values take precedence over higher values, 
 *  meaning a thread with priority 4 will be 
 *  given higher priority than a thread with 
 *  priority 7. Similarly, a thread with 
 *  priority -2 will have higher priority 
 *  than both a thread with priority 4 
 *  and a thread with priority 7."
 *
 */

// Prority values
#define LOGGER_PRIORITY     10      
#define CONTROLLER_PRIORITY 1
#define SAFETY_PRIORITY     0

/* -----------------------------------------------------------------
 * ENUMS
 * -----------------------------------------------------------------
 */

// To do : Add more here !
enum ThreadStatus {
    // OK
    CONTROL_OK,
    MEASURE_OK,
    SAFETY_OK,
    LOGGER_OK,

    // Warnings
    CONTROL_WRN,
    MEASURE_WRN,
    SAFETY_WRN,
    LOGGER_WRN,

    // Errors
    CONTROL_ERR,
    MEASURE_ERR,
    SAFETY_ERR,
    LOGGER_ERR
};

#endif /* _DEF_THREADS */