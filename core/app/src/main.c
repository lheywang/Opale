/*
 * Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>

#include <hal/nrf_gpio.h> 

#define LED_PIN1     NRF_GPIO_PIN_MAP(0,30)

LOG_MODULE_REGISTER(app);

int main(void)
{
	int cnt = 0;
    nrf_gpio_cfg_output(LED_PIN1);
    nrf_gpio_pin_set(LED_PIN1);

	while (1) {
		LOG_INF("test %d", cnt++);
		printk("Hello world from %s\n", CONFIG_BOARD);
        printk("Application core :)\n");

		k_msleep(500);
        nrf_gpio_pin_set(LED_PIN1);
        k_msleep(500);
        nrf_gpio_pin_clear(LED_PIN1);
	}

	return 0;
}
