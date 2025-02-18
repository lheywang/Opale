/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/peripherals/pwm.h
 *
 * @brief   Expose standard functions for the PWM usage on the 
 *          nRF5340 SoC.
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 * @date    15/02/2025
 *
 * @version 1.0.0
 *
 * ---------------------------------------------------------------
 */
#pragma once

// Include Nordic Semi HAL
#include <hal/nrf_pwm.h>

// STDLIB 
#include <stdint.h>

// ==============================================================
// Structs
// ==============================================================

/**
 * @brief   Struct to store the parameters about the PWM Peripheral.
 * 
 */
typedef struct {
    NRF_PWM_Type *p_PWM;
    uint8_t Pins[4];
    uint8_t Ports[4];
    uint8_t PinCount;
    uint8_t PeripheralID;
} PWM;

// ==============================================================
// Functions declaration
// ==============================================================

/**
 * @brief   Open a new PWM peripheral and enable it.
 *
 * @warning If the remaining memory isn't enough, the function will return
 *          -1, which will mostly cause the program to crash.
 *          This is intendend since the code can't execute correctly then, 
 *          and error can't be handled.
 * 
 * @return PWM :    A pointer to a PWM struct that will store data related 
 *                  to the peripheral.
 * @return -1 :     No PWM peripheral remaining.
 * @return -2 :     Can't allocate enough memory for the struct
 *
 */
PWM* PWM_Open();

/**
 * @brief   Stop, disable and close the peripheral.
 * 
 * @param Target    Pointer to a PWM struct to be closed.
 *
 * @return 0 :      PWM stopped and closed.
 */
int PWM_Close(PWM *Target);

/**
 * @brief   Add a pin and configure it to the PWM struct.
 *
 * @warning This function does not configure the pin directly, it will be done
 *          on PWM_Configure function call.
 * 
 * @param   Target  Pointer to a PWM struct 
 * @param   Port    Port number    
 * @param   Pin     Pin number 
 *
 * @return  0 :     Pin was added
 * @return -1 :     All pins are already used.
 */
int PWM_AddPin(PWM *Target, uint8_t Port, uint8_t Pin);

/**
 * @brief   Configure the PWM.
 * 
 * @param   Target  Pointer to a PWM struct
 * @param   clk     Clock enum member of the nRF HAL
 * @param   mode    Mode enum member of the nrf HAL
 * @param   max_val Maximal value for the counter
 *
 * @return  0 :     PWM Configured
 */
int PWM_Configure(PWM *Target, nrf_pwm_clk_t clk, nrf_pwm_mode_t mode, uint16_t max_val);

