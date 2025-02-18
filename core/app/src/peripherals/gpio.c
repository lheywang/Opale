/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/peripherals/gpio.h
 *
 * @brief   Define standard functions for the GPIO usage on the 
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
#include "gpio.h"

// Include Nordic Semi HAL
#include <hal/nrf_gpio.h>

// STDLIB 
#include <stdint.h>
#include <stdlib.h>

// ==============================================================
// Functions declaration
// ==============================================================

GPIO* GPIO_Open(uint8_t Port, uint8_t Pin, GPIOMode Mode){

    // Allocate a new struct and make sure everything is set to 0
    GPIO *GPIO = malloc(sizeof(GPIO));
    if (!GPIO)
        return -1;
    memset(GPIO, 0, sizeof(*GPIO));

    // Fill settings
    GPIO->Port = Port;
    GPIO->Pin = Pin;
    GPIO->Mode = Mode;

    // Initialize the GPIO in the correct direction
    if (Mode == GPIO_OUTPUT)
        nrf_gpio_cfg_output(
            NRF_GPIO_PIN_MAP(
                GPIO->Port, 
                GPIO->Pin
            )
        );

    // Return the struct
    return GPIO;
}

int GPIO_Close(GPIO *Target)
{
    free(Target);
    return 0;
}

int GPIO_Write(GPIO *Target, int Value){

    // Check if the value CAN be wrote
    if (Target->Mode == GPIO_INPUT)
        return -1;

    if (!Value){
        nrf_gpio_pin_set(
            NRF_GPIO_PIN_MAP(
                Target->Port, 
                Target->Pin
            )
        );
        return 0;
    }
    nrf_gpio_pin_clear(
        NRF_GPIO_PIN_MAP(
            Target->Port, 
            Target->Pin
        )
    );
    return 0;
}

int GPIO_Read(GPIO *Target, int *Value){
    // Check if the GPIO is configured as input or output :
    if (Target->Mode == GPIO_INPUT) {
        *Value = nrf_gpio_pin_read(
                    NRF_GPIO_PIN_MAP(
                        Target->Port, 
                        Target->Pin
                    )
                );
        return 0;
    }
    *Value = nrf_gpio_pin_out_read(
                NRF_GPIO_PIN_MAP(
                    Target->Port,
                    Target->Pin
                )
            );
    return 0;
}

int GPIO_Toggle(GPIO *Target){
    // CHeck if the pin can be toggled...
    if (Target->Mode == GPIO_INPUT)
        return -1;

    nrf_gpio_pin_toggle(
        NRF_GPIO_PIN_MAP(
            Target->Port,
            Target->Pin
        )
    );
    return 0;
}