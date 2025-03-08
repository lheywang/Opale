/**
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sub-license, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * File:   mcp23008_driver_basic.c
 * Author: Cedric Akilimali
 *
 * Created on August 04, 2022, 1:25 PM
 */

#include "mcp23008_driver_basic.h"

/**
 * @brief basic example initialize
 * @param[in] variant is the device type
 * @param[in] addr_pin is the i2c slave user-defined hardware address bits (A2,A1,A0)
 * @return status code
 *          - 0 success
 *          - 1 initialize failed
 * @note    none
 */
uint8_t mcp23008_basic_initialize(mcp23008_address_pin_t addr_pin) {

    volatile uint8_t err;
    volatile uint8_t index;

    /*link function*/
    DRIVER_MCP23008_LINK_INIT(&mcp23008_handle, mcp23008_handle_t);
    DRIVER_MCP23008_LINK_I2C_INIT(&mcp23008_handle, mcp23008_interface_i2c_init);
    DRIVER_MCP23008_LINK_I2C_DEINIT(&mcp23008_handle, mcp23008_interface_i2c_deinit);
    DRIVER_MCP23008_LINK_I2C_READ(&mcp23008_handle, mcp23008_interface_i2c_read);
    DRIVER_MCP23008_LINK_I2C_WRITE(&mcp23008_handle, mcp23008_interface_i2c_write);
    DRIVER_MCP23008_LINK_DELAY_MS(&mcp23008_handle, mcp23008_interface_delay_ms);
    DRIVER_MCP23008_LINK_DEBUG_PRINT(&mcp23008_handle, mcp23008_interface_debug_print);
    DRIVER_MCP23008_LINK_RECEIVE_CALLBACK(&mcp23008_handle, mcp23008_interface_receive_callback);

    /*mcp23008 initialize*/
    err = mcp23008_init(&mcp23008_handle);
    if (err) {
        return err;
    }

    /*set address pin*/
    err = mcp23008_set_addr_pin(&mcp23008_handle, addr_pin);
    if (err) {
        return err;
    }

    /*set interrupt pin logic level*/
    err = mcp23008_set_intrrupt_pin_output_level(&mcp23008_handle, MCP23008_interrupt_ACTIVE_LOW);
    if (err) {
        return err;
    }

    /*set interrupt pin mode*/
    err = mcp23008_set_interrupt_pin_output_mode(&mcp23008_handle, MCP23008_interrupt_OPEN_DRAIN_OUTPUT);
    if (err) {
        return err;
    }

    /*disable i2c slewrate by default*/
    err = mcp23008_set_slew_rate(&mcp23008_handle, MCP23008_BOOL_FALSE);
    if (err) {
        return err;
    }

    /* disable sequential mode*/
    err = mcp23008_set_sequencial_mode(&mcp23008_handle, MCP23008_BOOL_TRUE);
    if (err) {
        return err;
    }

    /*set interrupt compare mode*/
    for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        err = mcp23008_set_ineterrupt_compare_mode(&mcp23008_handle, index, MCP23008_interrupt_COMP_TO_DEFAULT_VALUE);
        if (err) {
            return err;
        }
    }

    /*set interrupt capture polarity */
    /*for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        err = mcp23008_set_pin_interrupt_caputure_level(&mcp23008_handle, index, MCP23008_interrupt_CAP_LOW);
        if (err) {
#ifdef MCP23008_DEBUG_MODE
            mcp23008_interface_debug_print("mcp23008: failed to set GP%d capture polarity\n\r", index);
#endif
            return 1;
        }
    }*/

    /*set pin polarity mode*/
    for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        err = mcp23008_set_pin_input_polarity_mode(&mcp23008_handle, index, MCP23008_POLARITY_SAME_LOGIC_STATE);
        if (err) {
            return err;
        }
    }

    /*disable all pull-up*/
    for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        err = (mcp23008_set_pin_pullup_mode(&mcp23008_handle, index, MCP23008_BOOL_FALSE) != 0);
        if (err) {
            return err;
        }
    }
    /*disable all interrupt*/
    for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        err = (mcp23008_set_pin_interrupt(&mcp23008_handle, index, MCP23008_BOOL_FALSE) != 0);
        if (err) {
            return err;
        }
    }

    /*clear all interrupt*/
    for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        err = (mcp23008_clear_interrupt_flag(&mcp23008_handle/*, index, MCP23008_BOOL_FALSE*/) != 0);
        if (err) {
            return err;
        }
    }

    /* wait 10 ms */
    mcp23008_interface_delay_ms(10);

    return 0; /**< Initialize success */
}

/**
 * @brief interrupt request handle callback function
 * @param[in] *irq_callback point to the callback function
 * @return status code
 *          - 0 success
 *          - 1 fail to run handler
 */
uint8_t mcp23008_basic_irq_handler(void) {

   err = mcp23008_irq_handler(&mcp23008_handle);
   return err; /**< return error code */
}

/**
 * @brief callback function to run interrupt service routine
 * @param[in] *irq_callback point to the interrupt routine function
 * @return status code
 *          - 0 succeed
 *          - 1 failed to run
 * @note none
 */
uint8_t mcp23008_basic_gpio_irq_callBack(mcp23008_irq_callback_t cb) {
   err = cb();
   return err; /**< return error code */
}

/**
 * @brief basic example to set gpio pin direction
 * @param[in] GPIOx is the gpio port to set direction
 * @param[in] direction is the desire direction to set the pin(input, output or input pull-up)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_gpio_set_direction(mcp23008_gpio_port_t GPIOx, mcp23008_port_mode_t direction) {
    uint8_t index;

    if (direction == MCP23008_INPUT_PULLUP) {
        if (mcp23008_set_pin_pullup_mode(&mcp23008_handle, GPIOx, MCP23008_BOOL_TRUE) != 0) {
            return 1; /**< failed to execute routine */
        }

        if (mcp23008_set_pin_mode(&mcp23008_handle, GPIOx, MCP23008_INPUT) != 0) {
            return 1; /**< failed to execute routine */
        }
    } else {
        if (mcp23008_set_pin_mode(&mcp23008_handle, GPIOx, direction) != 0) {
            return 1; /**< failed to execute routine */
        }
        for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) /**< clear port after setting the direction */
            mcp23008_basic_gpio_write(index, MCP23008_GPIO_LOW);
    }

    mcp23008_interface_delay_ms(50);
    return 0; /**< success */
}

/**
 * @brief basic example to set port direction
 * @param[in] direction is the desire direction to set the port(input, output or input pull-up)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_port_set_direction(mcp23008_port_mode_t direction)
{

    if (direction == MCP23008_INPUT_PULLUP) {
        err = mcp23008_set_port_pullup_mode(&mcp23008_handle, MCP23008_BOOL_TRUE);
        if(err != 0)
        {
            return err; /**< failed to execute routine */
        }

        err = mcp23008_set_port_mode(&mcp23008_handle, MCP23008_INPUT);
        if (err != 0)
        {
            return err; /**< failed to execute routine */
        }
    } else {
        err = mcp23008_set_port_mode(&mcp23008_handle, direction);
        if(err != 0)
        {
            return err; /**< failed to execute routine */
        }
    }
    return 0;
}

/**
 * @brief basic example to write logic value to gpio port
 * @param[in] GPIOx is the gpio port to write
 * @param[in] level is the logic level to write (HIGH or LOW)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_gpio_write(mcp23008_gpio_port_t GPIOx, mcp23008_port_logic_level_t level) {

    err = mcp23008_pin_write(&mcp23008_handle, GPIOx, level);
    return err; /**< return error code */
}

 /**
 * @brief basic example to write logic value to all gpio pins
 * @param[in] logic_level is the logic level to assign to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed to write pin
 * @note    none
 */
uint8_t mcp23008_basic_pin_write_all(mcp23008_port_logic_level_t logic_level)
{
    err = mcp23008_pin_write_all(&mcp23008_handle, logic_level);
    return err; /**< return error code */
}

/**
 * @brief basic example to read gpio port
 * @param[in] GPIOx is the gpio port to read
 * @return GPIO read status (pin level)
 * @note    none
 */
uint8_t mcp23008_basic_gpio_read(mcp23008_gpio_port_t GPIOx) {

    volatile uint8_t status;
    err = mcp23008_pin_read(&mcp23008_handle, GPIOx, (int *) &status);
    if(err != 0)
    {
       return err;
    }
    return status; /**< success */
}

/**
 * @brief basic example of gpio toggle function
 * @param[in] GPIOx is the port to toggle
 * @return status code
 *          - 0 success
 *          - 1 failed to toggle
 * @note    none
 */
uint8_t mcp23008_basic_gpio_toggle(mcp23008_gpio_port_t GPIOx) {

   err = mcp23008_basic_gpio_write(GPIOx, !mcp23008_basic_gpio_read(GPIOx));
   return err; /**< return error code */
}

/**
 * @brief basic example to enable interrupt
 * @param[in] GPIOx is the gpio port to enable interrupt
 * @param[in] edge_select is the Interrupt Edge Select bit (rising, or falling)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_interrupt_enable(mcp23008_gpio_port_t GPIOx, mcp23008_interrupt_default_value_t edge_select) {

    err = mcp23008_set_pin_interrupt(&mcp23008_handle, GPIOx, MCP23008_BOOL_TRUE);
    if(err)
    {
        return err; /**< failed to execute routine */
    }
    err = mcp23008_set_default_compare_value(&mcp23008_handle, GPIOx, edge_select);
    return err; /**< return error code */
}

/**
 * @brief basic example to disable interrupt
 * @param[in] GPIOx is the gpio port to disable interrupt
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_interrupt_disable(mcp23008_gpio_port_t GPIOx) {

   err = mcp23008_set_pin_interrupt(&mcp23008_handle, GPIOx, MCP23008_BOOL_FALSE);
   return err; /**< return error code */

}

/**
 * @brief basic example to clear interrupt flag
 * @param[in] GPIOx is the gpio port to clear interrupt flag
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_clr_interrupt_flag(void) {
   err = mcp23008_clear_interrupt_flag(&mcp23008_handle/*, GPIOx, MCP23008_interrupt_CLEAR*/);
   return err; /**< return error code */
}

/**
 * @brief basic exaple to get interrupt flag status
 * @param[in] GPIOx is the gpio port to clear interrupt flag
 * @param[out] *flag_status point to the interrupt flag status
 * @return interrupt status flag
 * @note    none
 */
uint8_t mcp23008_basic_get_interrupt_flag(mcp23008_gpio_port_t GPIOx, uint8_t *flag_status) {

    err = mcp23008_get_interrupt_flag(&mcp23008_handle, GPIOx, (uint8_t *) flag_status);
    return err; /**< return error code */
}

/**
 * @brief     get the address pin
 * @param[in] *addr_pin point to the chip address pins
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mcp23008_basic_get_addr_pin(mcp23008_address_pin_t *addr_pin)
{
	uint8_t status;

	err = mcp23008_get_addr_pin(&mcp23008_handle,(uint8_t *)&status);
	if(err != 0)
    {
        return err;
    }
	*addr_pin = status;

	return err;
}

/**
 * @brief basic example read register
 * @param[in] reg is the i2c register address
 * @param[out] *buf points to a data buffer read
 * @param[in] len is the data buffer length
 * @return status code
 *          - 0 success
 *          - 1 failed to read register
 */
uint8_t mcp23008_basic_read_register(uint8_t reg, uint8_t *buf, uint16_t len) {
    err = mcp23008_get_reg(&mcp23008_handle, reg, buf, len);
    return err; /**< return error code */
}

/**
 * @brief basic example write register
 * @param[in] reg is the i2c register address
 * @param[in] *buf points to a data buffer to write
 * @param[in] len is the data buffer length
 * @return status code
 *          - 0 success
 *          - 1 failed to write register
 */
uint8_t mcp23008_basic_write_register(uint8_t reg, uint8_t *buf, uint16_t len) {
   err = mcp23008_set_reg(&mcp23008_handle, reg, buf, len);
   return err; /**< return error code */
}
/*end*/
