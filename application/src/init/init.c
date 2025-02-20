/** ================================================================
 * @file    application/src/init/init.c
 *
 * @brief   Init.c define the function that check if the peripherals
 *          are ready to be used, or not.
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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

#include "init.h"
#include "../config.h"

/* -----------------------------------------------------------------
* LOGGER CONFIG
* -----------------------------------------------------------------
*/
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Initializer, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
* FUNCTIONS TO CHECK IF THE PERIPHERAL IS OK
* -----------------------------------------------------------------
*/
int CheckLedsPeripherals(){

    int ErrCounter = 0;

    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!gpio_is_ready_dt(&led)) {
            if (k < 2){
                LOG_WRN("LED peripheral is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("LED peripheral is not working properly.");
                ErrCounter += 1;
            }
        }
        else 
            break;
    }

    if (ErrCounter == 0)
        LOG_INF("Leds are working !");
    return -ErrCounter;
}

int CheckPWMPeripherals(){

    int ErrCounter = 0;

    for (uint8_t i = 0; i < PWM_SERVO_LEN; i++) {
        for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
            if (!pwm_is_ready_dt(&pwm_wings[i])) {
                if (k < 2){
                    LOG_WRN("Servo %d peripheral is not ready for now. Retrying in 5 ms...", i);
                    k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
                }
                else {
                    LOG_ERR("Servo %d peripheral is not working properly.", i);
                    ErrCounter += 1;
                }
            }
            else 
                break;
        }
        LOG_INF("Servo engine %d is working correctly...", i);
    }

    for (uint8_t i = 0; i < PWM_RGB_LEN; i++) {
        for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
            if (!pwm_is_ready_dt(&pwm_rgb[i])) {
                if (k < 2){
                    LOG_WRN("RGB Led %d peripheral is not ready for now. Retrying in 5 ms...", i);
                    k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
                }
                else {
                    LOG_ERR("RGB Led %d peripheral is not working properly.", i);
                    ErrCounter += 1;
                }
            }
            else 
                break;
        }
        LOG_INF("RGB Led %d is working correctly...", i);
    }

    if (ErrCounter == 0){
        LOG_INF("All PWM peripherals are working properly !");
    }
    else {
        LOG_ERR("Some PWM peripheralsaren't working properly... Err = %d", ErrCounter);
    }
    return -ErrCounter;
}