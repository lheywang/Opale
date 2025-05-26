/** ================================================================
 * @file    application/src/threads/controllerFunction/PID.cpp
 * 
 * @brief   PID.h declares the PID to control each control surfaces
 *
 * @date    26/05/2025
 *
 * @version 1.0.0
 *
 * @author  a.brandstaedt (arthur.brandstaedt@gmail.com)
 *
 *
 *  ================================================================
 */

// Include once
#ifndef DEF_PID
#define DEF_PID

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Libs
#include "init/init.hpp"
#include <arm_math.h>

/* -----------------------------------------------------------------
 * Defining command structure
 * -----------------------------------------------------------------
 */

/**
 * @brief   Defines the PID controller structure using the CMSIS-DSP library.
 */
arm_pid_instance_f32 PID[3];

/**
 * @brief   Defines and initiate the output matrix of the PID controller.
 */
float outputPID[3] = {0.0f, 0.0f, 0.0f}; // Initialize PID values to zero
arm_matrix_instance_f32 outputMatrix[3];
arm_mat_init_f32(outputMatrix, 3, 1, (float *)outputPID);

/**
 * @brief   Defines and initiate the control matrix for the angle output of the PID controller.
 */
float control[4][3] = {
//  pitch,  yaw,  roll
    {1.0f,  0.0f, 1.0f}, // Control surface 1
    {0.0f,  1.0f, 1.0f}, // Control surface 2
    {-1.0f, 0.0f, 1.0f}, // Control surface 3
    {0.0f, -1.0f, 1.0f}  // Control surface 3
};
arm_matrix_instance_f32 controlMatrix[3];
arm_mat_init_f32(controlMatrix, 3, 4, (float *)control);

/**
 * @brief   Defines and initiate the output angle matrix for the PID controller.
 */
float outputAngle[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // Initialize angles to zero
arm_matrix_instance_f32 angleMatrix[4];
arm_mat_init_f32(angleMatrix, 4, 1, (float *)outputAngle);

 /* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND A PID CONTROLLER
 * -----------------------------------------------------------------
 */
namespace PID
{
    /**
     * @brief   Initialize the PID controller with the given parameters.
     *
     * @param   pid     Pointer to the PIDController structure.
     * @param   Kp      Proportional gain array for the PID controller.
     * @param   Ki      Integral gain array for the PID controller.
     * @param   Kd      Derivative gain array for the PID controller.
     * 
     * @return  0   Value configured
     */
    int PID_Init_3Axis(arm_pid_instance_f32 pid[3], const float Kp[3], const float Ki[3], const float Kd[3]);

    /**
     * @brief   Compute the PID output based on the measured value.
     * 
     * @details This function limits the output to a range of -100 to 100
     *          to prevent excessive control signals. The output is
     *          the torque that should be applied to the control surface
     *          in newton-meters (Nm).
     *
     * @param   pid         Pointer to the PIDController structure.
     * @param   setpoint    The desired setpoint value for the PID controller.
     * @param   measured    The measured value to compare against the setpoint.
     * 
     * @return              The computed PID output value.
     */

    arm_matrix_instance_f32* PID_Compute_3Axis(arm_pid_instance_f32 pid[3], const float setpoint[3], const float measured[3]);
}





#endif /* DEF_PID*/