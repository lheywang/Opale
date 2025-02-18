/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/peripherals/pwm.c
 *
 * @brief   Define standard functions for the PWM usage on the 
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

// Include interface
#include "pwm.h"

// Include Nordic Semi HAL
#include <hal/nrf_pwm.h>
#include <hal/nrf_gpio.h>

// STDLIB 
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

PWM* PWM_Open(){

    // Handle the selection of PWM engine dynamically... (and return if no)
    static int PWM_Number = 0;
    static bool PWM_Enabled[4] = {false};

    PWM_Number += 1;
    if (PWM_Number > 4)
        return -1;

    // If we reach this point, then at least a PWM engine is available...
    int PWM_ID = 255;
    for (uint8_t k = 0; k < 4; k++)
    {
        if (PWM_Enabled[k] == false)
            PWM_ID = k;
            break;
    }
    PWM_Enabled[PWM_ID] = true;
    
    // Allocate a new struct and make sure everything is set to 0
    PWM *PWM = malloc(sizeof(PWM));
    if (!PWM)
        return -2;
    memset(PWM, 0, sizeof(*PWM));

    // Then, get a new pointer on the values...
    PWM->p_PWM = NRF_PWM_INST_GET(PWM_ID);
    PWM->PeripheralID = PWM_ID;

    // Enable PWM
    nrf_pwm_enable(PWM->p_PWM);

    // Exit and return a pointer to the allocated struct !
    return PWM;
}

int PWM_AddPin(PWM *Target, uint8_t Port, uint8_t Pin){
    // Check if a pin is remaining
    if (Target->PinCount > NRF_PWM_CHANNEL_COUNT)
        return -1;

    // Increment pincount
    Target->PinCount += 1;

    // Update values
    Target->Ports[Target->PinCount - 1] = Port;
    Target->Pins[Target->PinCount - 1] = Pin;

    return 0; 
}

int PWM_Close(PWM *Target){
    // First, stop and disable PWM
    nrf_pwm_disable(Target->p_PWM); 

    // Free memory
    free(Target);

    return 0;
}


int PWM_Configure(PWM *Target, nrf_pwm_clk_t clk, nrf_pwm_mode_t mode, uint16_t max_val){
    // First, transform the data into the correct form
    uint32_t pins_tmp [NRF_PWM_CHANNEL_COUNT] = {NRF_PWM_PIN_NOT_CONNECTED};
    for (uint8_t k = 0; k < Target->PinCount; k++){
        pins_tmp[k] = NRF_GPIO_PIN_MAP(
            Target->Ports[k], 
            Target->Pins[k]
        );
    }

    // Call the HAL to configure the pins...
    nrf_pwm_pins_set(Target->p_PWM, pins_tmp);

    // Configure other parameters...
    nrf_pwm_configure(
        Target->p_PWM, 
        clk, 
        mode, 
        max_val
    );
    return 0;
}