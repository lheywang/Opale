/** ================================================================
 * @file    application/src/threads/controllerFunction/PID.cpp
 * 
 * @brief   PID.cpp implement the PID to control each control surfaces
 *
 * @date    26/05/2025
 *
 * @version 1.0.0
 *
 * @author  a.brandstaedt (arthur.brandstaedt@gmail.com)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Header
#include "threads/controllerFunction/PID.h"

// Zephyr
#include <zephyr/logging/log.h>

// CMSIS-DSP
#include <arm_math.h>

// STD
#include <math.h>

// Local libs
#include "devices/rgb.h"
#include "config.h"
#include "init/init.hpp"


/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(PID, PROJECT_LOG_LEVEL);

 /* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND A PID CONTROLLER
 * -----------------------------------------------------------------
 */

int PID_Init_3Axis(arm_pid_instance_f32 pid[3], const float Kp[3], const float Ki[3], const float Kd[3])
{
    for (int i = 0; i < 3; ++i) {
        pid[i].Kp = Kp[i];
        pid[i].Ki = Ki[i];
        pid[i].Kd = Kd[i];
        arm_pid_init_f32(&pid[i], 1); // 1 = reset state value
    }

    return 0; // Return 0 to indicate successful initialization
}

arm_matrix_instance_f32* Control_Matrix_Init(const float controlConst[3][4]) {
    // Already defined in the header file
    extern float control[3][4];
    extern arm_matrix_instance_f32 controlMatrix[3];

    // Initialize the control matrix with the provided constants
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            control[i][j] = controlConst[i][j];
        }
    }

    arm_mat_init_f32(&controlMatrix, 3, 4, (float *)control);
    
    return (arm_matrix_instance_f32*)controlMatrix;
}

arm_matrix_instance_f32* PID_Compute_3Axis(arm_pid_instance_f32 pid[3], const float setpoint[3], const float measured[3])
{
    // Already defined in the header file
    extern float outputPID[3];
    extern arm_matrix_instance_f32 outputMatrix[3];

    extern float control[3];
    extern arm_matrix_instance_f32 controlMatrix[3];

    extern float outputAngle[3];
    extern arm_matrix_instance_f32 angleMatrix[3];

    for(int i = 0; i < 3; ++i) {
        // Compute the PID output for each axis
        outputPID[i] = arm_pid_f32(&pid[i], setpoint[i] - measured[i]);

        arm_mat_mult_f32(&controlMatrix[i], &outputMatrix[i], &angleMatrix[i]);

        if (outputAngle[i] > 10.0f) {
            outputAngle[i] = 10.0f; // Limit to max angle
            
        } else if (outputAngle[i] < -10.0f) {
            outputAngle[i] = -10.0f; // Limit to min angle
        }
    }

    return (arm_matrix_instance_f32*)angleMatrix;
}