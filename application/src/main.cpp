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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>

// Custom headers
#include "init/init.h"
#include "config.h"

#include "devices/servo.h"
#include "devices/rgb.h"
#include "devices/eeprom.h"

#include "peripherals/gpio.h"
#include "peripherals/saadc.h"

// Devices drivers
#include "drivers/MS5611.h"
#include "drivers/mcp23008.h"
#include "drivers/bno055.h"
#include "drivers/iis2dlpc_reg.h"
#include "drivers/teseo.h"

// Threads
#include "threads/controller.h"
#include "threads/logger.h"
#include "threads/safety.h"
#include "threads/threads.h"

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Main, PROJECT_LOG_LEVEL);

#define I2C_NODE DT_NODELABEL(barometer0)

K_THREAD_STACK_DEFINE(controller_stack, THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(logger_stack, THREAD_STACKSIZE);
K_THREAD_STACK_DEFINE(safety_stack, THREAD_STACKSIZE);

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

    // Fetch peripherals
    gpio_dt_spec *peripheral_reset = INIT_GetAGPIO(GPIOS::PERIPHERAL_RESET);
    pwm_dt_spec *pwm_wings = INIT_GetAPWM(PWMS::SERVOS);
    pwm_dt_spec *pwm_rgb = INIT_GetAPWM(PWMS::RGB);

    int ret = GPIO_SetAsOutput(peripheral_reset, 1);

    ret -= SAADC_Configure();

    if (ret < 0)
        return 0;

    // // This is working ! --> Due to the zephyr thread management, some task are executed way after !
    // MS5611 TempSensor = MS5611();
    // double *val = TempSensor.getPressure();
    // LOG_WRN("Vals %f %f", val[0], val[1]);

    // // This is working, we use the GPIO0. The second call always get an error, since it can't be used twice !
    // MCP23008 ExternalStart = MCP23008(MCP23008_GPIOS::GPIO0);
    // MCP23008 ExternalStart2 = MCP23008(MCP23008_GPIOS::GPIO0);
    // ExternalStart.readPin();
    // ExternalStart2.readPin();

    // Creating threads
    struct k_fifo toto;
    k_fifo_init(&toto);

    struct safety_p1 tmp1 = {.barom_data = toto,
                             .adc_data = toto,
                             .imu_data = toto,
                             .gps_data = toto,
                             .gpio_data = toto};
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

    struct safety_p1 safety_data = {    .barom_data = barom1,
                                        .adc_data = adc1,
                                        .imu_data = imu1,
                                        .gps_data = gps1,
                                        .accel_data = accels1,
                                        .controller = controller_thread,
                                        .logger = logger_thread,
                                        .measures = measure_thread};

    struct logger_p1 tmp2 = {.barom_data = toto,
                             .adc_data = toto,
                             .imu_data = toto,
                             .gps_data = toto};
    struct logger_p1 logger_data = {    .barom_data = barom3,
                                        .adc_data = adc3,
                                        .imu_data = imu3,
                                        .gps_data = gps2};

    struct controller_p1 tmp3 = {.barom_data = toto,
                                 .adc_data = toto,
                                 .imu_data = toto};
    struct controller_p1 controller_data = {    .barom_data = barom2,
                                                .adc_data = adc2,
                                                .imu_data = imu2,
                                                .accel_data = accels2};

    struct measure_p1 measure_data = {  .barom_data1 = barom1, .barom_data2 = barom2, .barom_data3 = barom3,
                                        .adc_data1 = adc1, .adc_data2 = adc2, .adc_data3 = adc3,
                                        .imu_data1 = imu1, .imu_data2 = imu2, .imu_data3 = imu3,
                                        .gps_data1 = gps1, .gps_data2 = gps2,
                                        .accels_data1 = accels1, .accels_data2 = accels2};

    k_thread_create(&controller_data,
                    controller_stack,
                    K_THREAD_STACK_SIZEOF(controller_stack),
                    thread_controller,
                    (void *)&tmp3,
                    nullptr,
                    nullptr,
                    CONTROLLER_PRIORITY,
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

    // This works fine !

    /* -----------------------------------------------------------------
     * INITIALIZING EXTERNAL DEVICES TO KNOWN POSITION
     * -----------------------------------------------------------------
     */

    ServoAngles Command = {.north = 90,
                           .south = 0,
                           .east = -90,
                           .west = 0};

    ret += SERVO_SetPosition(pwm_wings, &Command);

    Color Command2 = {.red = 0,
                      .green = 63,
                      .blue = 127,
                      .alpha = 50};

    ret += RGB_SetColor(pwm_rgb, &Command2);

    while (1)
    {
        Command.west = 1.5;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : 90 deg \n- South : 45 deg \n- East  : -17.5 deg \n- West  : 0.5 deg \n\n");
        k_msleep(500);
        GPIO_Toggle(peripheral_reset);

        Command.west = 0;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : 90 deg \n- South : 45 deg \n- East  : -17.5 deg \n- West  : 0 deg \n\n");
        k_msleep(500);
        GPIO_Toggle(peripheral_reset);
    }

    /* -----------------------------------------------------------------
     * MAIN LOOP
     * -----------------------------------------------------------------
     */

    while (1)
    {

        // Command 1
        Command.north = 90;
        Command.south = 45;
        Command.east = -17.5;
        Command.west = -50;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : 90 deg \n- South : 45 deg \n- East  : -17.5 deg \n- West  : -50 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 2
        Command.east = -12.5;
        Command.west = -10;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : 90 deg \n- South : 45 deg \n- East  : -12.5 deg \n- West  : -10 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 3
        Command.north = -90;
        Command.south = -45;
        Command.east = -7.5;
        Command.west = -30;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : -90 deg \n- South : -45 deg \n- East  : -7.5 deg \n- West  : -30 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 4
        Command.east = -2.5;
        Command.west = 10;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : -90 deg \n- South : -45 deg \n- East  : -2.5 deg \n- West  : 10 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 5
        Command.north = 90;
        Command.south = 45;
        Command.east = 2.5;
        Command.west = -10;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : 90 deg \n- South : 45 deg \n- East  : 2.5 deg \n- West  : -10 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 6
        Command.east = 7.5;
        Command.west = 30;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : 90 deg \n- South : 45 deg \n- East  : 7.5 deg \n- West  : 30 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 7
        Command.north = -90;
        Command.south = -45;
        Command.east = 12.5;
        Command.west = 10;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : -90 deg \n- South : -45 deg \n- East  : 12.5 deg \n- West  : 10 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);

        // Command 8
        Command.east = 17.5;
        Command.west = 50;
        ret += SERVO_SetPosition(pwm_wings, &Command);
        LOG_INF("Configured engines to : \n- North : -90 deg \n- South : -45 deg \n- East  : 17.5 deg \n- West  : 50 deg \n\n");
        k_msleep(2500);
        GPIO_Toggle(peripheral_reset);
    }
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

    for (;;) k_msleep(1000 * 1000 * 1000); // Sleep for 16 minutes, repeatable...

    /*
     * We shall never get here...
     */
    return 0;

}

/*
 * This sample make moves the engine position following different commands :
 * - north engine : alternate betweeen 90 and -90 every 5 seconds
 * - south engine : alternate between 46 and -45 every 5 seconds
 * - east engine : discrete ramp, +5 degree every 2.5 second. (with a final step of -80 degree to restart).
 * - west engine : random response : -50, -10, -30, 10, -10, 30, 10, 50 degrees to measure random response.
 *
 * - ADC shall print in console measurements per channel every 2.5 seconds to match the response.
 *
 *  */
