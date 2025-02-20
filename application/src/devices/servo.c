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
#include "../config.h"
#include "../init/init.h"

// STD
#include <math.h>

/* -----------------------------------------------------------------
* LOGGER CONFIG
* -----------------------------------------------------------------
*/
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Servo, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
* FUNCTIONS TO COMMAND A SERVO
* -----------------------------------------------------------------
*/

int ServosSetPosition(  const struct pwm_dt_spec Target[PWM_SERVO_LEN], 
                        ServoAngles const *Command){
    
    // Checking the parameters
    // MIN =< Pos <= MAX
    if ((Command->north < PWM_SERVO_MIN_ANGLE) || 
        (Command->south < PWM_SERVO_MIN_ANGLE) || 
        (Command->east < PWM_SERVO_MIN_ANGLE) || 
        (Command->west < PWM_SERVO_MIN_ANGLE)){
        LOG_ERR("Incorrect command passed on the servo. At least one command was under the threshold");
        return -1;
    }
    if ((Command->north > PWM_SERVO_MAX_ANGLE) || 
        (Command->south > PWM_SERVO_MAX_ANGLE) || 
        (Command->east > PWM_SERVO_MAX_ANGLE) || 
        (Command->west > PWM_SERVO_MAX_ANGLE)){
        LOG_ERR("Incorrect command passed on the servo. At least one command was under the threshold");
        return -1;
    }

    // Computing the pulses length
    // Due to negative values that are offseted, we need to handle that case too.
    int pulses[4] = {0};
    for (uint8_t k = 0; k < 4; k++)
    {
        switch (k) {
            case 1:
                pulses[k] = Command->north;
                break;
            case 2:
                pulses[k] = Command->south;
                break;
            case 3:
                pulses[k] = Command->east;
                break;
            case 4:
                pulses[k] = Command->west;
                break;
        }

        pulses[k] += PWM_SERVO_MAX_ANGLE;
        pulses[k] *= PWM_SERVO_MAX_PULSE_WIDTH;
        pulses[k] /= 2 * PWM_SERVO_MAX_RANGE;
        pulses[k] += PWM_SERVO_MIN_PULSE_WIDTH;
        pulses[k] = round(pulses[k]);
    }

    // Configure the PWM engines
    int err = 0;
    for (uint8_t k = 0; k < 4; k++)
    {
        err += pwm_set_dt(&Target[k], PWM_SERVO_PERIOD, pulses[k]);
    }

    // End log, and return
    if (err != 0){
        LOG_ERR("Failed to configure the PWM duty cycle for the servo : %d", err);
        return -2;
    }
    return 0;
}