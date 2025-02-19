/** ================================================================
 * @file    application/src/init/init.h
 *
 * @brief   Init.h is charged to fetch the DT node label from the
 *          devicetree and expose them to the C application code.
 *          This file also do some verification about the fetched 
 *          structure to ensure they're usable. 
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

// Include once
#ifndef DEF_INIT
    #define DEF_INIT

    /* -----------------------------------------------------------------
    * INCLUDING LIBS
    * -----------------------------------------------------------------
    */

    // Zephyr driver libs
    #include <zephyr/drivers/gpio.h>
    #include <zephyr/drivers/pwm.h>

    /* -----------------------------------------------------------------
    * DEFINING NODE ALIASES
    * -----------------------------------------------------------------
    */

    // Onboard leds
    #define LED0        DT_ALIAS(led0)

    // Servo engines (wings) commands
    #define SERVO_0     DT_ALIAS(servo1)
    #define SERVO_1     DT_ALIAS(servo2)
    #define SERVO_2     DT_ALIAS(servo3)
    // #define SERVO_3     DT_ALIAS(servo4)

    /* -----------------------------------------------------------------
    * FETCHING C STRUCTS THAT DESCRIBE EACH PERIPHERALS
    * -----------------------------------------------------------------
    */

    // Onboard leds
    static const struct gpio_dt_spec    led            = GPIO_DT_SPEC_GET(LED0, gpios);

    // Servo engines (wings) commands
    static const struct pwm_dt_spec     pwm0_servo0    = PWM_DT_SPEC_GET(SERVO_0);
    static const struct pwm_dt_spec     pwm0_servo1    = PWM_DT_SPEC_GET(SERVO_1);
    static const struct pwm_dt_spec     pwm0_servo2    = PWM_DT_SPEC_GET(SERVO_2);
    // static const struct pwm_dt_spec     pwm0_servo3    = PWM_DT_SPEC_GET(SERVO_3);

    /* -----------------------------------------------------------------
    * FUNCTIONS TO CHECK IF THE PERIPHERAL IS OK
    * -----------------------------------------------------------------
    */

    /**
     * @brief   This function check if all of the leds are working properly
     * 
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckLedsPeripherals();

    /**
     * @brief   This function check if all of the PWMs are working properly
     * 
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckPWMPeripherals();

#endif

