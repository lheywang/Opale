/** ================================================================
 * @file    application/src/devices/rgb.h
 *
 * @brief   rgb.h define some higher level interractions with the
 *          onboard RGB led.
 *          
 *
 * @date    20-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

// Include once
#ifndef DEF_RGB
    #define DEF_RGB

    /* -----------------------------------------------------------------
    * INCLUDING LIBS
    * -----------------------------------------------------------------
    */

    // Zephyr
    #include <zephyr/drivers/pwm.h>
    
    // Libs
    #include "../init/init.h"

    /* -----------------------------------------------------------------
    * FETCHING NODE PARAMETERS
    * -----------------------------------------------------------------
    */

    // Period
    #define PWM_RGB_PERIOD            DT_PROP(RGB_1, period)

    /* -----------------------------------------------------------------
    * Defining command structure
    * -----------------------------------------------------------------
    */

    /**
     * @brief   Define a structure used to pass a color command to the 
     *          RGB LED
     * 
     * @var     Color::red      The RGB code between 0-255 for the red.
     * @var     Color::green    The RGB code between 0-255 for the green.
     * @var     Color::blue     The RGB code between 0-255 for the blue.
     * @var     Color::alpha    An alpha value to set the max brightness.
     */
    typedef struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
        uint8_t alpha;
    } Color;

    /* -----------------------------------------------------------------
    * FUNCTIONS TO COMMAND AN RGB LED
    * -----------------------------------------------------------------
    */

    /**
     * @brief   Configure the PWM duty cycle to prompt a defined color 
     *          on the onboard RGB led.
     * 
     * @param   Target      The pointer to the struct that describe the leds
     * @param   Command     A pointer to a Color struct.
     * 
     * @return  0   Value configured
     * @return -1   Incorrect command passed (alpha field)
     * @return -2   Error while calling the OS
     */
    int LedSetColor(const struct pwm_dt_spec Target[PWM_RGB_LEN], 
                    Color const *Command);

#endif

