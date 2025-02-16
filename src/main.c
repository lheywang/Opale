#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#ifdef CONFIG_GPIOHANDLE
    #include "peripherals/gpio.h"
#endif

int main(void)
{
	while (1) {
		printk("Hello World!\n");
		k_msleep(1000);
	}
}

