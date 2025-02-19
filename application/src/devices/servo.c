/** ================================================================
 * @file    application/src/devices/servo.c
 *
 * @brief   Servo.c implement some low level interactions with the 
 *          servo engines.
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
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

// Local libs
#include "servo.h"

/* -----------------------------------------------------------------
* LOGGER CONFIG
* -----------------------------------------------------------------
*/
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Servo, LOG_LEVEL_DBG);

/* -----------------------------------------------------------------
* FUNCTIONS TO COMMAND A SERVO
* -----------------------------------------------------------------
*/

int SetServoPosition(const void* Target, float Position){
    
    // Checking the parameters
    if ((Position < PWM_SERVO_MIN_ANGLE) || (Position > PWM_SERVO_MAX_ANGLE)) 
        return -1;

    // Computing the values (cross product...)
    Position += PWM_SERVO_MIN_ANGLE; // Offset the value to get a 0-MAX+MIN 
    int pulse = (Position * PWM_SERVO_MAX_PULSE_WIDTH) / PWM_SERVO_MAX_ANGLE;

    // Configure the PWM engine
    int err = pwm_set_dt(Target, PWM_SERVO_PERIOD, pulse); // pulse is ns, check that ?
    if (err != 0)
        return -2;
    return 0;
}