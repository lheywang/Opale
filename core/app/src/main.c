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
#include <zephyr/logging/log.h>

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


#define LED_PIN1     NRF_GPIO_PIN_MAP(0,30)
#define LED_PIN2     NRF_GPIO_PIN_MAP(0,31)

LOG_MODULE_REGISTER(app);

int main(void)
{
	int cnt = 0;
    nrf_gpio_cfg_output(LED_PIN1);
    nrf_gpio_cfg_output(LED_PIN2);
    nrf_gpio_pin_set(LED_PIN1);
    nrf_gpio_pin_clear(LED_PIN2);

	while (1) {
		LOG_INF("test %d", cnt++);
		printk("Hello world from %s\n", CONFIG_BOARD);
        printk("Application core :)\n");

		k_msleep(500);
        nrf_gpio_pin_set(LED_PIN1);
        nrf_gpio_pin_clear(LED_PIN2);
        k_msleep(500);
        nrf_gpio_pin_clear(LED_PIN1);
        nrf_gpio_pin_set(LED_PIN2);
	}

	return 0;
}
