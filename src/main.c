#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>

#define LED0_NODE   DT_ALIAS(led0)          // LED0_NODE = led0 defined in the .dts file
#define SERVO_0     DT_ALIAS(servo1)        // PWM node 0
#define SERVO_1     DT_ALIAS(servo2)        // PWM node 1
#define SERVO_2     DT_ALIAS(servo3)        // PWM node 1

static const struct pwm_dt_spec     pwm1 = PWM_DT_SPEC_GET(SERVO_0);
static const struct pwm_dt_spec     pwm2 = PWM_DT_SPEC_GET(SERVO_1);
static const struct pwm_dt_spec     pwm3 = PWM_DT_SPEC_GET(SERVO_2);
static const struct gpio_dt_spec    led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{

    if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

    if (!pwm_is_ready_dt(&pwm1)) {
        return 0;
    }
    
    if (!pwm_is_ready_dt(&pwm2)) {
        return 0;
    }
    
    if (!pwm_is_ready_dt(&pwm3)) {
        return 0;
    }

    int ret;

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }
    ret = pwm_set_dt(&pwm1, PWM_MSEC(1), PWM_USEC(900));
    if (ret) {
        return 0;
    }
    ret = pwm_set_dt(&pwm2, PWM_MSEC(1), PWM_USEC(500));
    if (ret) {
        return 0;
    }
    ret = pwm_set_dt(&pwm3, PWM_MSEC(1), PWM_USEC(100));
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

