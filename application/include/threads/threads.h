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
#define THREAD_STACKSIZE 1024

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
#define LOGGER_PRIORITY 10
#define CONTROLLER_PRIORITY 1
#define SAFETY_PRIORITY 0

/* -----------------------------------------------------------------
 * ENUMS
 * -----------------------------------------------------------------
 */

/*
 *  This enum define an integer value, to be used on the messages
 *  queue.
 *  It define a "protocol" between the thread and the safety thread,
 *  that decide if a task shall be performed.
 *
 *  It may be seen as a duplicate of the status register, and, for the
 *  most basic usage, yes. At one exception : The status is only a
 *  basic data, as : The ADC is working !.
 *
 *  The messages are more like : The thread is alive, which is different,
 *  and only rode by the safety task. The safety task may cancel and
 *  reschedule any task, if needed (act as watchdog).
 *
 */
enum ThreadStatus
{
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

/* -----------------------------------------------------------------
 * GLOBALS STATUS
 * -----------------------------------------------------------------
 */
/*
 * Each bit of the 32 bits register define a status. They're defined
 * rigth above !
 *
 * ------------------------------------------------------------------
 * Bits 0-3 :   **Theses bits define the authorization to launch,
 *                as well as some actions during the flight**
 * 0 :      Safety status. Set to 1 if operationnal, 0 otherwise.
 * 1 :      Logger status. Set to 1 if operationnal, 0 otherwise.
 * 2 :      Measure status. Set to 1 if operationnal, 0 otherwise.
 * 3 :      Controller status. Set to 1 if operationnal, 0 otherwise.
 *
 * -------------------------------------------------------------------
 * Bits 4-10 :  **Theses bits define the status of available measures.
 *                They may change during the execution of the flight !**
 * 4 :      ADC status. Set to 1 if operationnal, 0 otherwise.
 * 5 :      GPS Status. Set to 1 if operationnal, 0 otherwise.
 * 6 :      Accelerometers. Set to 1 if operationnal, 0 otherwise.
 * 7 :      Barometer. Set to 1 if operationnal, 0 otherwise.
 * 8 :      GPIOS. Set to 1 if operationnal, 0 otherwise.
 * 9 :      Servos. Set to 1 if operationnal, 0 otherwise.
 * 10 :     SPI. Set to 1 if operationnal, 0 otherwise.
 *
 * -------------------------------------------------------------------
 *
 * Bits 30-31 : **This bit is only changed by the safety thread**
 * 30 :     Control enabled
 * 31 :     Launch enabled
 *
 */

// Bits 0-3
#define STATUS_THREAD_BASE 0
#define STATUS_SAFETY (1 << (STATUS_THREAD_BASE + 0))
#define STATUS_LOGGER (1 << (STATUS_THREAD_BASE + 1))
#define STATUS_MEASURES (1 << (STATUS_THREAD_BASE + 2))
#define STATUS_CONTROLLER (1 << (STATUS_THREAD_BASE + 3))

// Bits 4-10
#define STATUS_PERIPH_BASE 4
#define STATUS_ADC (1 << (STATUS_PERIPH_BASE + 0))
#define STATUS_GPS (1 << (STATUS_PERIPH_BASE + 1))
#define STATUS_ACCELS (1 << (STATUS_PERIPH_BASE + 2))
#define STATUS_BAROM (1 << (STATUS_THREAD_BASE + 3))
#define STATUS_GPIOS (1 << (STATUS_PERIPH_BASE + 4))
#define STATUS_SERVOS (1 << (STATUS_PERIPH_BASE + 5))
#define STATUS_SPI (1 << (STATUS_PERIPH_BASE + 6))

// Bits 30-31
#define STATUS_AUTH_BASE 30
#define STATUS_AUTH_LAUNCH (1 << (STATUS_AUTH_BASE + 0))
#define STATUS_AUTH_CONTROL (1 << (STATUS_AUTH_BASE + 1))

// TEST
struct fifo_data
{
    void *reserved;
    int value;
};

#endif /* _DEF_THREADS */