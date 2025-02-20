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

    for (uint8_t k = 0; k < 3; k++) {
        if (!gpio_is_ready_dt(&led)) {
            if (k < 2){
                LOG_WRN("LED peripheral is not ready for now...");
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

    return -ErrCounter;
}

int CheckPWMPeripherals(){

    int ErrCounter = 0;

    for (uint8_t k = 0; k < 3; k++) {
        if (!pwm_is_ready_dt(&pwm0_servo0)) {
            if (k < 2){
                LOG_WRN("PWM1 peripheral is not ready for now...");
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

    for (uint8_t k = 0; k < 3; k++) {
        if (!pwm_is_ready_dt(&pwm0_servo1)) {
            if (k < 2){
                LOG_WRN("LED peripheral is not ready for now...");
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

    for (uint8_t k = 0; k < 3; k++) {
        if (!pwm_is_ready_dt(&pwm0_servo2)) {
            if (k < 2){
                LOG_WRN("LED peripheral is not ready for now...");
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

    // for (uint8_t k = 0; k < 3; k++) {
    //     if (!pwm_is_ready_dt(&pwm0_servo3)) {
    //         if (k < 2){
    //             LOG_WRN("LED peripheral is not ready for now...");
    //             k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
    //         }
    //         else {
    //             LOG_ERR("LED peripheral is not working properly.");
    //             ErrCounter += 1;
    //         }
    //     }
    //     else 
    //         break;
    // }

    return -ErrCounter;
}