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

    // Motor specs (and compute the correct values)
    #define PWM_SERVO_MAX_RANGE         DT_PROP(SERVO_0, max_angle)
    #define PWM_SERVO_MAX_ANGLE         (PWM_SERVO_MAX_RANGE / 2)
    #define PWM_SERVO_MIN_ANGLE         (-PWM_SERVO_MAX_RANGE / 2)

    /* -----------------------------------------------------------------
    * FUNCTIONS TO COMMAND A SERVO
    * -----------------------------------------------------------------
    */

    /**
     * @brief   Compute the PWM duty cycle needed to place a servo engine
     *          in a defined position.
     * 
     * @details This function compute, in the backend a cross product to get
     *          the wanted pulse duration over the standard period.
     * 
     *          This indeed does not integrate any form of feedback nor precision,
     *          we're handling floating points values that are rounded at the end.
     *          
     *          The servo can expose a deadband where a slight modification of the
     *          pulse lenght won't change the position. A call to this function may in 
     *          this case initiate a little to no movement.
     * 
     *          The value produced are :
     * 
     *                     Angle  | Pulse len
     *              ------------- | -------------
     *                        -90 | MIN_PULSE
     *                         90 | MAX_PULSE
     * 
     * 
     * @param   Target      The pwm_dt_spec that correspond to the servo 
     *                      (defined as const)
     * @param   Position    The position in degrees that is wanted.
     * 
     * @return  0 : Controlled servo position
     * @return -1 : Invalid angle
     * @return -2 : Error while controlling the peripheral
     */
    int SetServoPosition(const void* Target, const float Position);

#endif

