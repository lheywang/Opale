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

// STD
#include <math.h>

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

int SetServoPosition(const void* Target, const float Position){
    
    // Checking the parameters
    // MIN =< Pos <= MAX
    if (Position < PWM_SERVO_MIN_ANGLE){
        LOG_ERR("Incorrect command passed on the servo %.3f", (double)Position);
        return -1;
    }
    if (Position > PWM_SERVO_MAX_ANGLE){
        LOG_ERR("Incorrect command passed on the servo %.3f", (double)Position);
        return -1;
    }

    // Computing the pulse length
    // Due to negative values that are offseted, we need to handle that case too.
    int pulse = Position;
    pulse += PWM_SERVO_MAX_ANGLE;
    pulse *= PWM_SERVO_MAX_PULSE_WIDTH;
    pulse /= 2 * PWM_SERVO_MAX_RANGE;
    pulse += PWM_SERVO_MIN_PULSE_WIDTH;
    pulse = round(pulse);

    // Configure the PWM engine
    int err = pwm_set_dt(Target, PWM_SERVO_PERIOD, pulse); // pulse is ns, check that ?
    if (err != 0){
        LOG_ERR("Failed to configure the PWM duty cycle for the servo : %d", err);
        return -2;
    }
    return 0;
}