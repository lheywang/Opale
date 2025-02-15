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
#include "adc.h"

// Include Nordic Semi HAL
#include <hal/nrf_gpio.h>