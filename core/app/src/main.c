/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/main.c
 *
 * @brief   Entry point code for the application core !
 *          This code applies for both secure and non-
 *          secure firmware !
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 * @date    15/02/2025
 *
 * @version 1.0.0
 *
 * ---------------------------------------------------------------
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <hal/nrf_gpio.h> 

// tests for includes
#include "config.h"
#include "peripherals/adc.h"
#include "peripherals/gpio.h"
#include "peripherals/i2c.h"
#include "peripherals/pwm.h"
#include "peripherals/spi.h"
#include "peripherals/uart.h"
#include "devices/BNO055.h"
#include "devices/II2SDLPC.h"
#include "devices/LIV3R.h"
#include "devices/M95256.h"
#include "devices/MS5611.h"

int main(void)
{
    GPIO *LED1 = GPIO_Open(0, 30, GPIO_OUTPUT);
    GPIO *LED2 = GPIO_Open(0, 31, GPIO_OUTPUT);

    GPIO_Write(LED1, 1);
    GPIO_Write(LED2, 0);

    PWM *PWM1 = PWM_Open();
    PWM_AddPin(PWM1, 0, 28);
    PWM_AddPin(PWM1, 0, 29);
    PWM_Configure(PWM1, NRF_PWM_CLK_500kHz, NRF_PWM_MODE_UP, 255);

	while (1) {
		k_msleep(500);
        GPIO_Toggle(LED1);
        GPIO_Toggle(LED2);
	};

    GPIO_Close(LED1);
    GPIO_Close(LED2);

    PWM_Close(PWM1);

	return 0;
}
