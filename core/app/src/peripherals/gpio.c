/*
 * ---------------------------------------------------------------
 * 
 * core/app/src/peripherals/gpio.h
 *
 * Define standard functions for the GPIO usage on the nRF5340 SoC.
 *
 * Warning : Theses function are customized for our board, and thus 
 * may not be specified for a specific peripheral.
 *
 * l.heywang
 * 15/02/2025
 *
 * TER M1 SEME 24-25
 *
 * ---------------------------------------------------------------
 */

// Include interface
#include "adc.h"

// Include Nordic Semi HAL
#include <hal/nrf_gpio.h>