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
    GPIO LED1 = GPIO_Open(0, 30, GPIO_OUTPUT);
    GPIO LED2 = GPIO_Open(0, 31, GPIO_OUTPUT);

    GPIO_Write(&LED1, 1);
    GPIO_Write(&LED2, 1);

	while (1) {
		printk("Hello world from %s\n", CONFIG_BOARD);
        printk("Application core :)\n");

	}

	return 0;
}
