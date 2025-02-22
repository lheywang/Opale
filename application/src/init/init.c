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
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
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
                    LOG_WRN("Servo %d (wings) peripheral is not ready for now. Retrying in 5 ms...", i);
                    k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
                }
                else {
                    LOG_ERR("Servo %d (wings) peripheral is not working properly.", i);
                    ErrCounter += 1;
                }
            }
            else {
                LOG_INF("Servo engines (wings) %d is working correctly...", i);
                break;
            }
        }
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
            else {
                LOG_INF("RGB Led %d is working correctly...", i);
                break;
            }
        }       
    }

    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!pwm_is_ready_dt(&pwm_parachute)) {
            if (k < 2){
                LOG_WRN("Parachute peripheral is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("Parachute peripheral is not working properly.");
                ErrCounter += 1;
            }
        }
        else {
            LOG_INF("Parachute is working correctly...");
            break;
        }
    } 

    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!pwm_is_ready_dt(&pwm_buzzer)) {
            if (k < 2){
                LOG_WRN("Buzzer peripheral is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("Buzzer peripheral is not working properly.");
                ErrCounter += 1;
            }
        }
        else {
            LOG_INF("Buzzer is working correctly...");
            break;
        }
    } 

    if (ErrCounter == 0){
        LOG_INF("All PWM peripherals are working properly !");
    }
    else {
        LOG_ERR("Some PWM peripheralsaren't working properly... Err = %d", ErrCounter);
    }
    return -ErrCounter;
}



int CheckUARTPeripherals(){

    int ErrCounter = 0;

    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!device_is_ready(uart_gps)) {
            if (k < 2){
                LOG_WRN("GPS (UART) is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("GPS (UART) is not working properly.");
                ErrCounter += 1;
            }
        }
        else {
            LOG_INF("GPS (UART) is working correctly...");
            break;
        }
    }  

    
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!device_is_ready(uart_imu)) {
            if (k < 2){
                LOG_WRN("IMU (UART) is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("IMU (UART) is not working properly.");
                ErrCounter += 1;
            }
        }
        else {
            LOG_INF("IMU (UART) is working correctly...");
            break;
        }
    } 

    if (ErrCounter == 0){
        LOG_INF("All UARTS peripherals are working properly !");
    }
    else {
        LOG_ERR("Some UARTS peripheralsaren't working properly... Err = %d", ErrCounter);
    }
    return -ErrCounter;
}

int CheckI2CPeripherals(){

    int ErrCounter = 0;

    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!device_is_ready(i2c_barometer.bus)) {
            if (k < 2){
                LOG_WRN("Barometer peripheral (I2C) is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("Barometer peripheral (I2C) is not working properly.");
                ErrCounter += 1;
            }
        }
        else {
            LOG_INF("Barometer (I2C) is working correctly...");
            break;
        }
    } 

    for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
        if (!device_is_ready(i2c_expander.bus)) {
            if (k < 2){
                LOG_WRN("Barometer peripheral (I2C) is not ready for now. Retrying in 5 ms...");
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else {
                LOG_ERR("Barometer peripheral (I2C) is not working properly.");
                ErrCounter += 1;
            }
        }
        else {
            LOG_INF("Barometer (I2C) is working correctly...");
            break;
        }
    }

    for (uint8_t i = 0; i < ACCEL_NB; i++)
    {
        for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
            if (!device_is_ready(i2c_accels[i].bus)) {
                if (k < 2){
                    LOG_WRN("Accelerometer %d peripheral (I2C) is not ready for now. Retrying in 5 ms...", i);
                    k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
                }
                else {
                    LOG_ERR("Accelerometer %d peripheral (I2C) is not working properly.",i);
                    ErrCounter += 1;
                }
            }
            else {
                LOG_INF("Accelerometer %d (I2C) is working correctly...", i);
                break;
            }
        }
    }

    if (ErrCounter == 0){
        LOG_INF("All I2C peripherals are working properly !");
    }
    else {
        LOG_ERR("Some I2C peripheralsaren't working properly... Err = %d", ErrCounter);
    }
    return -ErrCounter;
}

// int CheckSPIPeripherals(){

//     int ErrCounter = 0;

//     for (uint8_t i = 0; i < EEPROM_NB; i ++)
//     {
//         for (uint8_t k = 0; k < INIT_MAX_TRY; k++) {
//             if (!spi_is_ready_dt(&spi_eeproms[i])) {
//                 if (k < 2){
//                     LOG_WRN("EEPROM %d peripheral (SPI) is not ready for now. Retrying in 5 ms...", i);
//                     k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
//                 }
//                 else {
//                     LOG_ERR("EEPROM %d peripheral (SPI) is not working properly.", i);
//                     ErrCounter += 1;
//                 }
//             }
//             else {
//                 LOG_INF("EEPROM %d peripheral (SPI)is working correctly...", i);
//                 break;
//             }
//         } 
//     }

//     if (ErrCounter == 0){
//         LOG_INF("All SPI peripherals are working properly !");
//     }
//     else {
//         LOG_ERR("Some SPI peripheralsaren't working properly... Err = %d", ErrCounter);
//     }
//     return -ErrCounter;

// }