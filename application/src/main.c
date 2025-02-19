/** ================================================================
 * @file    application/src/main.c
 *
 * @brief   This file is the entry point for the application core. 
 *          It handle the initialization of all the tasks, and then
 *          launch child threads to handle for each one they're task.
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

/* -----------------------------------------------------------------
* INCLUDING LIBS
* -----------------------------------------------------------------
*/

// Zephyr
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>


// Custom headers
#include "init/init.h"
#include "devices/servo.h"

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

const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));

/* -----------------------------------------------------------------
* MAIN LOOP
* -----------------------------------------------------------------
*/
int main(void)
{
    // Checking that all of the peripherals were functionnal
    int err = 0;
    err += CheckLedsPeripherals();
    err += CheckPWMPeripherals();

    if (!device_is_ready(uart)){
		printk("UART device not ready\r\n");
		return 1 ;
	}

    static uint8_t tx_buf[] =   {"nRF Connect SDK Fundamentals Course\r\n"
        "Press 1-3 on your keyboard to toggle LEDS 1-3 on your development kit\r\n"};

    int ret = uart_tx(uart, tx_buf, sizeof(tx_buf), SYS_FOREVER_US);
	if (ret) {
		return 1;
	}

    if (err != 0)
        return 0;

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return 0;
    }

    // Configure PWM position of servos
    ret += SetServoPosition(&pwm0_servo0, -45);
    ret += SetServoPosition(&pwm0_servo1, 45);
    ret += SetServoPosition(&pwm0_servo2, 0);

    // error here :
    // servo aren't controlled, because of the expansion of a value incorrecly.

	while (1) {
		printk("Hello World!\n");
		k_msleep(1000);

        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            return 0 ;
        }
	}
}

