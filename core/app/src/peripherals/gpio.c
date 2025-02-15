/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/peripherals/gpio.h
 *
 * @brief   Define standard functions for the GPIO usage on the 
 *          nRF5340 SoC.
 *
 * @warning Theses function are customized for our board, and thus 
 *          may not be specified for a specific peripheral.
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
#include "gpio.h"

// Include Nordic Semi HAL
#include <hal/nrf_gpio.h>

// ==============================================================
// Functions declaration
// ==============================================================

GPIO GPIO_Open(uint8_t Port, uint8_t Pin, GPIOMode Mode){
    // Allocate a new struct and fill it with 0.
    GPIO = (GPIO*)malloc(1 * sizeof(GPIO));
    memset(GPIO, 0, sizeof(GPIO));

    // Fill settings
    GPIO.Port = Port;
    GPIO.Pin = Pin;
    GPIO.Mode = Mode;

    // Return the struct
    return GPIO;
}

int GPIO_Close(GPIO *Target)
{
    return 1;
}

int GPIO_Write(GPIO *Target, int Value){
    return 1;
}

int GPIO_Read(GPIO *Target, int *Value){
    return 1;
}