#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#include "init/init.h"

/* -----------------------------------------------------------------
* INCLUDING VARIABLES
* -----------------------------------------------------------------
*/
// Onboard leds
extern const struct     gpio_dt_spec    led;

// Servo PWM
extern const struct     pwm_dt_spec     pwm0_servo0;
extern const struct     pwm_dt_spec     pwm0_servo1;
extern const struct     pwm_dt_spec     pwm0_servo2;

int main(void)
{
    // Checking that all of the peripherals were functionnal
    int err = 0;
    err += CheckLedsPeripherals();
    err += CheckPWMPeripherals();

    if (err != 0)
        return 0;

    int ret;

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    ret = pwm_set_dt(&pwm0_servo0, PWM_MSEC(1), PWM_USEC(900));
    if (ret) {
        return 0;
    }
    ret = pwm_set_dt(&pwm0_servo1, PWM_MSEC(1), PWM_USEC(500));
    if (ret) {
        return 0;
    }
    ret = pwm_set_dt(&pwm0_servo2, PWM_MSEC(1), PWM_USEC(100));
    if (ret) {
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

