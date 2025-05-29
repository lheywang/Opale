/** ================================================================
 * @file    application/src/threads/controller/controller.cpp
 *
 * @brief   This file implement the main controller of the rocket.
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

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */
// Header
#include "threads/controller.h"

// Internal libs
#include "config.h"
#include "init/init.hpp"
#include "threads/threads.h"
#include "devices/servo.h"
#include "threads/controllerFunction/PID.h"
#include "threads/controllerFunction/parachute.h"

// Zephyr
#include <zephyr/logging/log.h>

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(CONTROLLER, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * THREADS
 * -----------------------------------------------------------------
 */

void thread_controller(void *p1, void *p2, void *p3)
{
    /* -----------------------------------------------------------------
     * Initialization
     * -----------------------------------------------------------------
     */
    // init phase : Fetching arguments
    __unused struct controller_p1 *IO = (controller_p1 *)p1;
    struct k_event *globalStatus = (k_event *)p2;
    struct k_msgq *threadStatus = (k_msgq *)p3;

    // Wait for the order of boot to be valid :
    bool wait = true;
    while (wait == true)
    {
        uint32_t val = k_event_test(globalStatus, STATUS_MEASURES);
        wait = (val != 0) ? false : true;
        k_msleep(20);
    }

    // Getting handles to output peripherals
    struct pwm_dt_spec *pwm_wings = initializer::GetAPWM(PWMS::SERVOS);
    struct pwm_dt_spec *pwm_para = initializer::GetAPWM(PWMS::PARACHUTE);
    struct gpio_dt_spec *engines = initializer::GetAGPIO(GPIOS::ENGINES);

    // Setting up some variables...
    // Gain for PID controller with KP, KI and KD respectively
    float K[3][3] = {
        {1.0f, 0.1f, 1.0f}, // Pitch PID
        {1.0f, 0.1f, 1.0f},  // Yaw PID
        {1.0f, 0.1f, 1.0f}   // Roll PID
    }; 
    
    // Initialize the PID controllers for each axis
    arm_pid_instance_f32 PID_controller_3axis[3];

    for (int i = 0; i < 3; ++i)
    {
        PID_Init_3Axis(PID_controller_3axis[i], K[i][1], K[i][2], K[i][3]);
    }

    // Initialize the output matrix for the PID controller
    float control_matrix[4][3] = {
    //  pitch,   yaw, roll
        {1.0f,  0.0f, 1.0f}, // Control surface 1 (= North)
        {0.0f,  1.0f, 1.0f}, // Control surface 2 (= West)
        {-1.0f, 0.0f, 1.0f}, // Control surface 3 (= South)
        {0.0f, -1.0f, 1.0f}  // Control surface 4 (= East)
    };

    Control_Matrix_Init(control_matrix);

    // Initialize the average window for the meaning of data
    constexpr int MEAN_WINDOW_SIZE = 10; // Size of the mean window
    constexpr int ADC_POSITION_SIZE = 4; // Number of control surfaces

    // Initialize the mean pressure array
    float barometerBuffer[MEAN_WINDOW_SIZE] = {0.0f}; // Array to hold the mean pressure values
    int barometerIndex = 0; // Index for the mean window
    int barometerCount = 0; // Count of valid pressure values in the mean window
    float barometerSum = 0.0f; // Sum of the pressure values in the mean window

    // Initializer the mean adc array
    float adcBuffer[ADC_POSITION_SIZE][MEAN_WINDOW_SIZE] = {0.0f}; // Array to hold the mean ADC values
    int adcIndex[ADC_POSITION_SIZE] = {0}; // Index for the mean ADC window
    int adcCount[ADC_POSITION_SIZE] = {0}; // Count of valid ADC values in the mean window
    float adcSum[ADC_POSITION_SIZE] = {0.0f}; // Sum of the ADC values in the mean window

    //We do not mean the IMU and the accelerometers data because of the Kalman filter

    static int initPressureBool = false; // Flag to indicate if the initial pressure has been set
    float initialPressure = 0.0f; // Initial pressure at launch

    // Liftoff variables and thresholds
    bool liftoff = false; // Flag to indicate if the rocket has lifted off
    constexpr float LIFTOFF_PRESSURE_DIFF_THRESHOLD = 20.0f; // in meters
    constexpr float LIFTOFF_ACCELERATION_THRESHOLD = 4.0f; // in m.s^-2 equals to 4g
    

    // Set ready bit before enterring main loop
    int thread_rval = ThreadStatus::CONTROL_OK;
    k_event_set_masked(globalStatus, 0xFFFFFFFF, STATUS_CONTROLLER);
    k_msgq_put(threadStatus, &thread_rval, K_MSEC(50));

    /* -----------------------------------------------------------------
     * Loop
     * -----------------------------------------------------------------
     */

    for (;;)
    {
        k_msleep(100); // Sleep for 100ms => to be changed with the sample time

        // Acquisition of the data
        // Get the barometer data from the FIFO
        baromData *barometer = (baromData *)k_fifo_get(&IO->barom_data, k_usec(100));
        if (barometer == NULL)
        {
            LOG_ERR("Failed to get barometer data from FIFO");
            continue; // Skip this iteration if no data is available
        } 

        // Get the ADC data from the FIFO
        adcData *adc = (adcData *)k_fifo_get(&IO->adc_data, k_usec(100));
        if (barometer == NULL)
        {
            LOG_ERR("Failed to get barometer data from FIFO");
            continue; // Skip this iteration if no data is available
        }

        // Get the IMU data from the FIFO
        imuData *imu = (imuData *)k_fifo_get(&IO->imu_data, k_usec(100));
        if (imu == NULL)
        {
            LOG_ERR("Failed to get IMU data from FIFO");
            continue; // Skip this iteration if no data is available
        }

        // Get the auxiliary accelerometer data from the FIFO
        accelData *accel = (accelData *)k_fifo_get(&IO->accel_data, k_usec(100));
        if (accel == NULL)
        {
            LOG_ERR("Failed to get auxiliary accelerometer data from FIFO");
            continue; // Skip this iteration if no data is available
        }

        // Meaning of the barometer data
        barometerSum -= barometerBuffer[barometerIndex]; // Subtract the oldest value from the sum

        barometerBuffer[barometerIndex] = barometer->pressure; // Update the buffer with the new pressure value
        barometerSum += barometer->pressure; // Add the new pressure value to the sum

        barometerIndex = (barometerIndex + 1) % MEAN_WINDOW_SIZE; // Update the index for the next iteration

        if (barometerCount < MEAN_WINDOW_SIZE) barometerCount += 1; // Increment the count of valid pressure values

        float barometerAVG = barometerSum / barometerCount; // Calculate the average pressure

        // Meaning of the ADC data for each control surface
        for(int i = 0; i < ADC_POSITION_SIZE; ++i)
        {
            adcSum[i] -= adcBuffer[i][adcIndex[i]]; // Subtract the oldest value from the sum

            adcBuffer[i][adcIndex[i]] = adc->pos[i]; // Update the buffer with the new ADC value
            adcSum[i] += adc->pos[i]; // Add the new ADC value to the sum

            adcIndex[i] = (adcIndex[i] + 1) % MEAN_WINDOW_SIZE; // Update the index for the next iteration

            if (adcCount[i] < MEAN_WINDOW_SIZE) adcCount[i] += 1; // Increment the count of valid ADC values
        }

        float adcAVG[ADC_POSITION_SIZE]; // Array to hold the average ADC values
        for(int i = 0; i < ADC_POSITION_SIZE; ++i)
        {
            adcAVG[i] = adcSum[i] / adcCount[i]; // Calculate the average ADC value for each control surface
        }

        if (!initPressureBool)
        {
            initPressureBool = true; // Set the initial pressure flag
            initialPressure = barometer->pressure; // Set the initial pressure at launch
        }  

        // Check for liftoff
        if (!liftoff && (barometer->pressure - LIFTOFF_PRESSURE_DIFF_THRESHOLD) < initialMeanPressure && imu->acc[1] > LIFTOFF_ACCELERATION_THRESHOLD)
        {
            liftoff = true; // Set liftoff flag
            LOG_INF("Liftoff detected!"); // Log liftoff event
        }

        arm_matrix_instance_f32* output = PID_Compute_3Axis(PID_controller_3axis, setpoint, measured);

    }
    return;
}