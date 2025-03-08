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
 * File:   mcp23008_driver_basic.h
 * Author: Cedric Akilimali
 *
 * Created on August 04, 2022, 1:25 PM
 */

#ifndef MCP23008_DRIVER_BASIC_H_INCLUDED
#define MCP23008_DRIVER_BASIC_H_INCLUDED

#include "mcp23008_driver_interface.h"

/**
 * @brief basic example initialize
 * @param[in] variant is the device type
 * @param[in] addr_pin is the i2c slave user-defined hardware address bits (A2,A1,A0)
 * @return status code
 *          - 0 success
 *          - 1 initialize failed
 * @note    none
 */
uint8_t mcp23008_basic_initialize(mcp23008_address_pin_t addr_pin);

 /**
 * @brief interrupt request handle callback function
 * @param[in] *irq_callback point to the callback function
 * @return status code
 *          - 0 success
 *          - 1 fail to run handler
 */
uint8_t mcp23008_basic_irq_handler(void);

/**
 * @brief callback function to run interrupt service routine
 * @param[in] *irq_callback point to the interrupt routine function
 * @return status code
 *          - 0 succeed
 *          - 1 failed to run
 * @note none
 */
uint8_t mcp23008_basic_gpio_irq_callBack(uint8_t (*irq_callback)(void));

/**
 * @brief basic example to set gpio pin direction
 * @param[in] GPIOx is the gpio port to set direction
 * @param[in] direction is the desire direction to set the pin(input, output or input pull-up)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_gpio_set_direction(mcp23008_gpio_port_t GPIOx, mcp23008_port_mode_t direction);

/**
 * @brief basic example to set port direction
 * @param[in] direction is the desire direction to set the port(input, output or input pull-up)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_port_set_direction(mcp23008_port_mode_t direction);

/**
 * @brief basic example to write logic value to gpio port
 * @param[in] GPIOx is the gpio port to write
 * @param[in] level is the logic level to write (HIGH or LOW)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
 uint8_t mcp23008_basic_gpio_write(mcp23008_gpio_port_t GPIOx, mcp23008_port_logic_level_t level);

 /**
 * @brief basic example to write logic value to all gpio pins
 * @param[in] logic_level is the logic level to assign to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed to write pin
 * @note    none
 */
uint8_t mcp23008_basic_pin_write_all(mcp23008_port_logic_level_t logic_level);

/**
* @brief basic example to read gpio port
* @param[in] GPIOx is the gpio port to read
* @return GPIO read status
* @note    none
*/
 uint8_t mcp23008_basic_gpio_read(mcp23008_gpio_port_t GPIOx);

 /**
 * @brief basic example of gpio toggle function
 * @param[in] GPIOx is the port to toggle
 * @return status code
 *          - 0 success
 *          - 1 failed to toggle
 * @note    none
 */
uint8_t mcp23008_basic_gpio_toggle(mcp23008_gpio_port_t GPIOx);

 /**
 * @brief basic example to enable interrupt
 * @param[in] GPIOx is the gpio port to enable interrupt
 * @param[in] edge_select is the Interrupt Edge Select bit (rising, or falling)
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_interrupt_enable(mcp23008_gpio_port_t GPIOx, mcp23008_interrupt_default_value_t edge_select);


 /**
 * @brief basic example to disable interrupt
 * @param[in] GPIOx is the gpio port to disable interrupt
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    none
 */
uint8_t mcp23008_basic_interrupt_disable(mcp23008_gpio_port_t GPIOx);

   /**
 * @brief basic example to clear interrupt flag
 * @return status code
 *          - 0 success
 *          - 1 failed
 * @note    This function should not be called in the interrupt callback routine
 */
uint8_t mcp23008_basic_clr_interrupt_flag(void);

/**
 * @brief basic example to get interrupt flag status
 * @param[in] GPIOx is the gpio port to clear interrupt flag
 * @param[out] *flag_status point to the interrupt flag status
 * @return interrupt status flag
 * @note    none
 */
uint8_t mcp23008_basic_get_interrupt_flag(mcp23008_gpio_port_t GPIOx, uint8_t *flag_status);

/**
 * @brief     get the address pin
 * @param[in] *addr_pin point to the chip address pins
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mcp23008_basic_get_addr_pin(mcp23008_address_pin_t *addr_pin);

/**
 * @brief basic example read register
 * @param[in] reg is the i2c register address
 * @param[out] *buf points to a data buffer read
 * @param[in] len is the data buffer length
 * @return status code
 *          - 0 success
 *          - 1 failed to read register
 */
  uint8_t mcp23008_basic_read_register(uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief basic example write register
 * @param[in] reg is the i2c register address
 * @param[in] *buf points to a data buffer to write
 * @param[in] len is the data buffer length
 * @return status code
 *          - 0 success
 *          - 1 failed to write register
 */
  uint8_t mcp23008_basic_write_register(uint8_t reg, uint8_t *buf, uint16_t len);

#endif // MCP23008_DRIVER_BASIC_H_INCLUDED
