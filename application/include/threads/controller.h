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

// Barometer data
struct barometerData{
    void *reserved;
    float pressure;
    float temperature;
};

// ADC data for control surfaces
struct adcData{
    void *reserved;
    float pos[4];   // Position of control surfaces
};

//Main IMU data
struct imuData{
    void *reserved;
    float acc[3];   // Accelerometer data   [X, Y, Z]           in m/s^2
    float gyro[3];  // Gyroscope data       [wX, wY, wZ]        in degrees/s
    float euler[3]; // Euler angles         [roll, pitch, yaw]  in degrees
    float mag[3];   // Magnetometer data    [x, y, z]           in microteslas
};

// Auxiliary accelerometer data
struct accelData{
    void *reserved;
    float auxAcc[3][2];
    // /!\ Still need to fuse the data /!\ 
};

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