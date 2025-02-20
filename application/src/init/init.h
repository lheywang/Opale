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
    #include <zephyr/devicetree.h>

    /* -----------------------------------------------------------------
    * DEFINING NODE ALIASES
    * -----------------------------------------------------------------
    */

    // Onboard leds
    #define LED0        DT_ALIAS(led0)

    // PWM related peripherals
    #define WINGS_1     DT_PATH(wings, wings1)
    #define RGB_1       DT_PATH(rgb, rgb1)

    // Serial related peripherals
    #define UART_0      DT_NODELABEL(uart0)

    // Fetching lengh of different DT PROP
    #define PWM_RGB_LEN               DT_PROP_LEN(RGB_1, pwms)
    #define PWM_SERVO_LEN             DT_PROP_LEN(WINGS_1, pwms)

    // Module settings
    #define INIT_MAX_TRY              3 // Number of try before declaring peripheral out.

    /* -----------------------------------------------------------------
    * FETCHING C STRUCTS THAT DESCRIBE EACH PERIPHERALS
    * -----------------------------------------------------------------
    */

    // Onboard leds
    static const struct gpio_dt_spec    led                      =      GPIO_DT_SPEC_GET(LED0, gpios);
    // UARTS
    // static const struct device          *uart                    =      DEVICE_DT_GET(UART_0);

    // Servo engines (wings) commands
    static const struct pwm_dt_spec     pwm_wings[PWM_SERVO_LEN] = {    PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 0),
                                                                        PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 1),
                                                                        PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 2),
                                                                        PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 3)};

    // RGB Leds
    static const struct pwm_dt_spec     pwm_rgb[PWM_RGB_LEN]     = {    PWM_DT_SPEC_GET_BY_IDX(RGB_1, 0), 
                                                                        PWM_DT_SPEC_GET_BY_IDX(RGB_1, 1), 
                                                                        PWM_DT_SPEC_GET_BY_IDX(RGB_1, 2)};



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

