#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>

#define LED0_NODE DT_ALIAS(led0) // LED0_NODE = led0 defined in the .dts file

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{

    if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

    int ret;

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

	while (1) {
		printk("Hello World!\n");
		k_msleep(1000);

        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0 ;
        }
	}
}

