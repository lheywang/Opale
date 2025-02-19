/** ================================================================
 * @file    application/src/devices/servo.h
 *
 * @brief   servo.h define some higher level interractions with the
 *          servos, for example configuring their positions, or 
 *          reading back the actual position.
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

// Include once
#ifndef DEF_SERVO
    #define DEF_SERVO

    /* -----------------------------------------------------------------
    * INCLUDING LIBS
    * -----------------------------------------------------------------
    */
    
    // Libs
    #include "../init/init.h"

    /* -----------------------------------------------------------------
    * FETCHING NODE PARAMETERS
    * -----------------------------------------------------------------
    */

    // Period
    #define PWM_SERVO_PERIOD            DT_PROP(SERVO_0, period)

    // Pulse specs
    #define PWM_SERVO_MIN_PULSE_WIDTH   DT_PROP(SERVO_0, min_pulse)
    #define PWM_SERVO_MAX_PULSE_WIDTH   DT_PROP(SERVO_0, max_pulse)

    // Motor specs
    #define PWM_SERVO_MIN_ANGLE         DT_PROP(SERVO_0, min_angle)
    #define PWM_SERVO_MAX_ANGLE         DT_PROP(SERVO_0, max_angle)

    /* -----------------------------------------------------------------
    * FUNCTIONS TO COMMAND A SERVO
    * -----------------------------------------------------------------
    */

    /**
     * @brief   Compute the PWM duty cycle needed to place a servo engine
     *          in a defined position.
     * 
     * @param   Target      The pwm_dt_spec that correspond to the servo 
     *                      (defined as const)
     * @param   Position    The position in degrees that is wanted.
     * 
     * @return  0 : Controlled servo position
     * @return -1 : Invalid angle
     * @return -2 : Error while controlling the peripheral
     */
    int SetServoPosition(const void* Target, float Position);

#endif

