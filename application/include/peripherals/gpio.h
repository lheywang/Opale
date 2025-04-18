/** ================================================================
 * @file    application/src/peripherals/gpio/gpio.h
 *
 * @brief   rgb.h define some higher level interractions with the
 *          base GPIO which can be quite complicated.
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
#ifndef DEF_GPIO
#define DEF_GPIO

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/drivers/gpio.h>

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND A GPIO
 * -----------------------------------------------------------------
 */

// Place inside a namespace
namespace gpio {
    /**
     * @brief   Set a defined GPIO as input.
     *
     * @warning This function take account of hardware inversion of pins,
     *          that may been done in the device tree.
     *
     * @param   Target  The GPIO to be configured
     *
     * @return  0 Pin configured
     * @return !0 Error
     */
    int SetAsInput(const struct gpio_dt_spec *Target);

    /**
     * @brief   Set a defined GPIO as output.
     *
     * @warning This function take account of hardware inversion of pins,
     *          that may been done in the device tree.
     *
     * @param   Target  The GPIO to be configured
     * @param   State   The state taken by the GPIO right after
     *
     * @return  0 Pin configured
     * @return !0 Error
     */
    int SetAsOutput(const struct gpio_dt_spec *Target, const uint8_t State);

    /**
     * @brief
     *
     * @warning This function take account of hardware inversion of pins,
     *          that may been done in the device tree.
     *
     * @param   Target  The GPIO to be configured
     *
     * @return  0 Pin configured
     * @return !0 Error
     */
    int SetAsHighZ(const struct gpio_dt_spec *Target);

    /**
     * @brief
     *
     * @warning This function take account of hardware inversion of pins,
     *          that may been done in the device tree.
     *
     * @param   Target  The GPIO to be toggled.
     *
     * @return  0 Pin configured
     * @return !0 Error
     */
    int Toggle(const struct gpio_dt_spec *Target);

    /**
     * @brief
     *
     * @warning This function take account of hardware inversion of pins,
     *          that may been done in the device tree.
     *
     * @param   Target  The GPIO to be set
     * @param   Value   The value to be wrote on the GPIO
     *
     * @return  0 Pin configured
     * @return !0 Error
     */
    int Set(const struct gpio_dt_spec *Target, const int Value);

    /**
     * @brief
     *
     * @warning This function take account of hardware inversion of pins,
     *          that may been done in the device tree.
     *
     * @param   Target  The GPIO to be read
     * @param   Value   Pointer to an integer to be filled.
     *
     * @return  0 Always 0.
     */
    int Read(const struct gpio_dt_spec *Target, int *const Value);
}

#endif /* DEF_GPIO*/
