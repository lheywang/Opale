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
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>


// Custom headers
#include "init/init.h"
#include "config.h"

#include "devices/servo.h"
#include "devices/rgb.h"
#include "devices/gpio.h"

/* -----------------------------------------------------------------
* LOGGER CONFIG
* -----------------------------------------------------------------
*/
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Main, PROJECT_LOG_LEVEL);


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
    err += CheckUARTPeripherals();
    err += CheckI2CPeripherals();
    // err += CheckSPIPeripherals();

    if (err != 0)
        return 0;

    int ret = GPIOSetAsOutput(&led, 0);
    if (ret < 0) {
        return 0;
    }

    ServoAngles Command = { .north = 66, 
                            .south = 33, 
                            .east = -33, 
                            .west = -66};

    ret += ServosSetPosition(pwm_wings, &Command);

    Color Command2 = {  .red = 255,
                        .green = 127,
                        .blue = 0,
                        .alpha = 100};
                
    ret += LedSetColor(pwm_rgb, &Command2);

	while (1) {
		LOG_INF("Hello World !");
		k_msleep(500);

        ret = GPIOToggle(&led);
        if (ret < 0) {
            return 0 ;
        }
	}
}

