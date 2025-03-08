/**
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
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
 * File:   mcp23008_driver.c
 * Author: Cedric Akilimali
 *
 * Created on August 04, 2022, 1:25 PM
 */

#include "mcp23008_driver.h"

/**
 * @brief chip information definition
 */
#define CHIP_NAME                "MCP23008"             /**< chip name for i2c version */
#define INTERFACE                "I2C"                  /**< interface protocol */
#define MANUFACTURER_NAME         "Microchip Technology" /**< manufacturer name */
#define SUPPLY_VOLTAGE_MIN        0.3f                   /**< chip min supply voltage */
#define SUPPLY_VOLTAGE_MAX        5.5f                   /**< chip max supply voltage */
#define MAX_CURRENT               150.0f                 /**< chip max current (mA)*/
#define MIN_CURRENT               120.0f                 /**< chip min current (mA)*/
#define TEMPERATURE_MIN           -40.0f                 /**< chip min operating temperature (�C) */
#define TEMPERATURE_MAX           125.0f                 /**< chip max operating temperature (�C) */
#define DRIVER_VERSION            1000                   /**< driver version */

/**
 * @brief i2c or spi write one byte
 * @param[in] *handle points to mcp23008 handle structure
 * @param[in] reg is the i2c or spi register address
 * @param[in] buffer is to the ata to write
 * @return status code
 *          - 0 success
 *          - 1 failed to write
 * @note none
 */

uint8_t a_mcp23008_i2c_write(mcp23008_handle_t *const pHandle, uint8_t u8Reg, uint8_t *pBuf, uint8_t u8Length) {

    uint8_t temp_buffer[u8Length + 1];
	temp_buffer[0] = u8Reg;
	for(int index = 1; index < u8Length + 1; index++){
		temp_buffer[index] = pBuf[index - 1];
	}

    if (pHandle->i2c_write(pHandle->i2c_address, (uint8_t *)temp_buffer, (u8Length + 1)) != 0) /**< write data */ {
            return 1; /**< write fail */
        }
    return 0; /**< success */
}

/**
 * @brief i2c or spi read one byte
 * @param[in] *handle points to mcp23008 handle structure
 * @param[in] reg is i2c or spi register address
 * @param[out] *buffer point to the ata read
 * @return status code
 *          - 0 success
 *          - 1 failed to read
 * @note none
 */
uint8_t a_mcp23008_i2c_read(mcp23008_handle_t *const pHandle, uint8_t u8Reg, uint8_t *pBuf, uint8_t u8Length) {

    if (pHandle->i2c_write(pHandle->i2c_address, (uint8_t *)&u8Reg, 1) != 0) /**< write reg */ {
            return 1; /**< write fail */
        }

    if (pHandle->i2c_read(pHandle->i2c_address, pBuf, u8Length) != 0) /**< read data */ {
            return 1; /**< read fail */
        }
    return 0;
}

/**
* @brief This function prints the error message
* @param[in] *pHandle points to mcp23008 handle structure
* @param[in] *pBuffer point to the string to be printed
* @return none
* @note   none
*/
void a_mcp23008_print_error_msg(mcp23008_handle_t *const pHandle, char *const pBuffer)
{
#ifdef MCP23008_DEBUG_MODE
    pHandle->debug_print("MCP23008: failed to %s.\r\n", pBuffer);
#endif // mcp23008_DEBUG_MODE
}

/**
 * @brief     Initialize the chip
 * @param[in] *handle points to mcp23008 handle structure
 * @return status code
 *            - 0 success
 *            - 1 i2c or spi initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 * @note      none
 */
uint8_t mcp23008_init(mcp23008_handle_t *const pHandle) {

    if (pHandle == NULL)
        return 2;
    if (pHandle->debug_print == NULL)
        return 3;

    if (pHandle->receive_callback == NULL) {
        pHandle->debug_print("mcp23008: receive_callback\n");
        return 3;
    }
    if (pHandle->delay_ms == NULL) {
        pHandle->debug_print("mcp23008: delay_ms\n");
    }

    if (pHandle->i2c_init == NULL) {
        pHandle->debug_print("mcp23008: i2c initialize is null\n");
        return 3;
    }

    if (pHandle->i2c_deinit == NULL) {
        pHandle->debug_print("mcp23008: i2c_deint is null\n");
        return 3;
    }

    if (pHandle->i2c_read == NULL) {
        pHandle->debug_print("mcp23008: i2c_read is null\n");
        return 3;
    }
    if (pHandle->i2c_write == NULL) {
        pHandle->debug_print("mcp23008: i2c_write is null\n");
        return 3;
    }
    if (pHandle->i2c_init()) {
        pHandle->debug_print("mcp23008: i2c initialize failed\n");
        return 1;
    }

    pHandle->inited = 1; /* flag complete initialization */

    return 0;
}

/**
 * @brief     close the chip
 * @param[in] *handle points to mcp23008 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 i2c or spi deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mcp23008_deinit(mcp23008_handle_t *const pHandle) {
    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    if (pHandle->i2c_deinit() != 0) /**< de-initialize i2c */ {
            a_mcp23008_print_error_msg(pHandle,"i2c deinit failed");
            return 1;
        }

    pHandle->inited = 0; /**< close driver */
    return 0; /**< success return 0 */
}

/**
 * @brief     irq handler
 * @param[in] *handle points to mcp23008 handle structure
 * @return  status code
 *            - 0 success
 *            - 1 run failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mcp23008_irq_handler(mcp23008_handle_t *const pHandle) {

    uint8_t err;
    uint8_t status;
    uint8_t index;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_FLAG_REG, (uint8_t*) &status,1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "failed to execute irq routine");
        return 1;
    }
    for (index = 0; index < MCP23008_MAX_NUM_GPIO_PIN; index++) {
        if (status & (0x01 << index)) {
            pHandle->receive_callback(index);
        }
    }

    return 0;

}

/**
 * @brief     set the address pin
 * @param[in] *handle points to a mcp23008 handle structure
 * @param[in] address_pin is the chip address pins
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mcp23008_set_addr_pin(mcp23008_handle_t *const pHandle, mcp23008_address_pin_t address_pin)
{
    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    pHandle->i2c_address = (MCP23008_ADDRESS_DEFAULT | address_pin);

    return 0;
}

/**
 * @brief     get the address pin
 * @param[in] *handle points to a mcp23008 handle structure
 * @param[in] *addr_pin point to the chip address pins
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mcp23008_get_addr_pin(mcp23008_handle_t *const pHandle, mcp23008_address_pin_t *addr_pin) {

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */


    *addr_pin = (mcp23008_address_pin_t) ((pHandle->i2c_address |MCP23008_ADDRESS_DEFAULT));

    return 0;
}

/**
 * @brief set pin mode (input, output or input_pullup)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configure
 * @param[in] mode is the mode gpio port to set
 * @return status code
 *          - 0 success
 *          - 1 failed to set pin mode
 * @note   none
 */
uint8_t mcp23008_set_pin_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_mode_t mode) {

    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_DIRECTION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set pin mode");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (mode << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_DIRECTION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
       a_mcp23008_print_error_msg(pHandle, "set pin mode");
        return 1;
    }
    return 0;
}

/**
 * @brief get pin mode (input, output or input_pullup)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configure
 * @param[out] *mode point to the gpio port mode set
 * @return status code
 *          - 0 success
 *          - 1 failed to get pin mode
 * @note   none
 */
uint8_t mcp23008_get_pin_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_mode_t *mode) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */
    err = a_mcp23008_i2c_read(pHandle, MCP23008_DIRECTION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read pin mode register");
        return 1;
    }

    *mode = (mcp23008_port_mode_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set port mode all pins to either (input, output or input_pullup)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] mode is the mode gpio port to set
 * @return status code
 *          - 0 success
 *          - 1 failed to set pin mode
 * @note   none
 */
uint8_t mcp23008_set_port_mode(mcp23008_handle_t *const pHandle, mcp23008_port_mode_t mode)
{
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    if(mode == MCP23008_INPUT)
        status = 0xff;
    else if(mode == MCP23008_OUTPUT)
        status = 0x00;

     err = a_mcp23008_i2c_write(pHandle, MCP23008_DIRECTION_REG, (uint8_t *) &status, 1);
        if (err != 0) {
           a_mcp23008_print_error_msg(pHandle, "set port mode");
            return 1;
        }

    return 0;
}

/**
 * @brief Write logic value to gpio pin
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to write (GP0 - GP7)
 * @param[in] logic_level is the logic level to assign to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed to write pin
 * @note    none
 */
uint8_t mcp23008_pin_write(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_logic_level_t logic_level) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_OUTPUT_LATCH_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read latch register");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (logic_level << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_OUTPUT_LATCH_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "write pin");
        return 1;
    }
    return 0;
}


/**
 * @brief Write logic value to all gpio pins
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] logic_level is the logic level to assign to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed to write pin
 * @note    none
 */
uint8_t mcp23008_pin_write_all(mcp23008_handle_t *const pHandle, mcp23008_port_logic_level_t logic_level)
{
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    if(logic_level == HIGH)
    {
        status = 0xff;
    }else if(logic_level == LOW){
        status = 0x00;
    }

    err = a_mcp23008_i2c_write(pHandle, MCP23008_OUTPUT_LATCH_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "write all pins");
        return 1;
    }

   return 0;
}

/**
 * @brief Read logic level of gpio pin
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to read (GP0 - GP7)
 * @param[out] *logic_level point to the logic level assigned to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed read pin
 * @note    none
 */
uint8_t mcp23008_pin_read(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_logic_level_t *logic_level) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_GENERAL_PURPOSE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read latch register");
        return 1;
    }
    *logic_level = (mcp23008_port_logic_level_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set pin pull-up mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to enable or disable pull-up mode
 * @param[in] enable is the boolean logic to set
 * @return status code
 *          - 0 success
 *          - 1 failed failed to set pull-up mode
 * @note    none
 */
uint8_t mcp23008_set_pin_pullup_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t enable) {

    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_GPIO_PULLUP_REG, (uint8_t *)&status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read pull-up register");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (enable << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_GPIO_PULLUP_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set pin pull-up");
        return 1;
    }
    return 0;
}

/**
 * @brief set port pull-up mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] enable is the boolean logic to set
 * @return status code
 *          - 0 success
 *          - 1 failed failed to set pull-up mode
 * @note    none
 */
uint8_t mcp23008_set_port_pullup_mode(mcp23008_handle_t *const pHandle, mcp23008_bool_t enable)
{
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    if(enable == MCP23008_BOOL_TRUE)
        status  = 0xff;
    else if(enable == MCP23008_BOOL_FALSE)
        status = 0x00;

    err = a_mcp23008_i2c_write(pHandle, MCP23008_GPIO_PULLUP_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set port pull-up");
        return 1;
    }
    return 0;

}


/**
 * @brief get pin pull-up mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to enable or disable pull-up mode
 * @param[out] *enable point to the boolean logic level
 * @return status code
 *          - 0 success
 *          - 1 failed get pull-up mode
 * @note    none
 */
uint8_t mcp23008_get_pin_pullup_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t *enable) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_GPIO_PULLUP_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read pull-up register");
        return 1;
    }
    //    status &= pinMask[pin];
    *enable = (mcp23008_bool_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set pin input polarity mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] polarity is the polarity value to assign
 * @return status code
 *          - 0 success
 *          - 1 failed to set polarity
 * @note    none
 */
uint8_t mcp23008_set_pin_input_polarity_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_polarity_t polarity) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INPUT_POLARITY_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read input polarity");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (polarity << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_INPUT_POLARITY_REG, (uint8_t *) &status, 1);
    if (err != 0) {
       a_mcp23008_print_error_msg(pHandle, "set pin input polarity");
        return 1;
    }
    return 0;
}

/**
 * @brief get pin input polarity mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[out] *polarity point to the polarity value to assigned to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed to get polarity
 * @note    none
 */
uint8_t mcp23008_get_pin_input_polarity_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_polarity_t *polarity) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INPUT_POLARITY_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read pin polarity");
        return 1;
    }
    //    status &= pinMask[pin];
    *polarity = (mcp23008_port_polarity_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set pin interrupt on change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] enable is the boolean logic value to set
 * @return status code
 *          - 0 success
 *          - 1 failed failed to set interrupt
 * @note    none
 */
uint8_t mcp23008_set_pin_interrupt(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t enable) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_ON_CHANGE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read interrupt");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (enable << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_INTERRUPT_ON_CHANGE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set pin interrupt");
        return 1;
    }
    return 0;
}

/**
 * @brief get pin interrupt on change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[out] *enable point to the boolean logic value set
 * @return status code
 *          - 0 success
 *          - 1 failed to get interrupt status
 * @note    none
 */
uint8_t mcp23008_get_pin_interrupt(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t *enable) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_ON_CHANGE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
       a_mcp23008_print_error_msg(pHandle,"read pin interrupt status");
        return 1;
    }
    //    status &= pinMask[pin];
    *enable = (mcp23008_bool_t) ((status & pinMask[pin]) >> pin);
    return 0; /**< success */
}

/**
 * @brief get pin interrupt flag
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[out] *flag point to interrupt flag status
 * @return status code
 *          - 0 success
 *          - 1 failed to get interrupt flag status
 * @note    none
 */
uint8_t mcp23008_get_interrupt_flag(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_flag_t *flag) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_FLAG_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read pin interrupt status");
        return 1;
    }

    *flag = (mcp23008_interrupt_flag_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief clear pin interrupt flag
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] flag is the interrupt flag
 * @return status code
 *          - 0 success
 *          - 1 failed to get interrupt flag status
 * @note    none
 */
uint8_t mcp23008_clear_interrupt_flag(mcp23008_handle_t *const pHandle/*, mcp23008_gpio_port_t pin, mcp23008_interrupt_flag_t flag*/) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_FLAG_REG, (uint8_t *) &status, 1);
    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_CAPTURED_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read interrupt flag");
        return 1;
    }
    /*status &= ~(1 << pin);
    status |= (flag << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_INTERRUPT_FLAG_REG, (uint8_t *) &status, 1);
    if(err != 0)
    {
        pHandle->debug_print("mcp23008: failed to clear pin interrupt flag\n\r");
        return 1;
    }*/
    return 0;
}

/**
 * @brief set interrupt output pin level when an interrutp occure
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] polarity is the interrupt pin level
 * @return status code
 *          - 0 success
 *          - 1 failed to set interrupt logic output logic level
 * @note    none
 */
uint8_t mcp23008_set_intrrupt_pin_output_level(mcp23008_handle_t *const pHandle, mcp23008_interrupt_polarity_t logic_level) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read expander reg");
        return 1;
    }
    status &= ~(1 << 1);
    status |= (logic_level << 1);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"interrupt pin output level");
        return 1;
    }
    return 0;
}

/**
 * @brief get interrupt output pinlevel assigned when an interrutp occure
 * @param[in] *handle point to mcp23008 handle structure
 * @param[out] *polarity is the interrupt pin level
 * @return status code
 *          - 0 success
 *          - 1 failed to get interrupt logic output logic level
 * @note    none
 */
uint8_t mcp23008_get_interrupt_pin_output_level(mcp23008_handle_t *const pHandle, mcp23008_interrupt_polarity_t *polarity) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read pin interrupt status");
        return 1;
    }
    *polarity = (mcp23008_interrupt_polarity_t) ((status & MCP23008_INTPOL_MASK) >> 1);
    return 0;
}

/**
 * @brief set gpio pin polarity level when interrupt occure
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] logic_level is the polarity level that will reflect on the pin when there is an interrupt
 * @return status code
 *          - 0 success
 *          - 1 failed set pin polarity level
 * @note    none
 */
//uint8_t mcp23008_set_pin_interrupt_caputure_level(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_captured_state_t logic_level)
//{
//    uint8_t err;
//    uint8_t status;
//
//    if (pHandle == NULL)     /**< check handle */
//        return 2;           /**< return error */
//    if(pHandle->inited != 1) /**< check initialize status */
//        return 3;           /**< return error */
//
//    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_CAPTURED_REG, (uint8_t *)&status);
//    if(err != 0)
//    {
//        pHandle->debug_print("mcp23008: failed to read interrupt capture register\n\r");
//        return 1;
//    }
//    status &= ~(1 << pin);
//    status |= (logic_level << pin);
//    err = a_mcp23008_i2c_write(pHandle, MCP23008_INTERRUPT_CAPTURED_REG, (uint8_t *) &status, 1);
//    if(err != 0)
//    {
//        pHandle->debug_print("mcp23008: failed set interrupt capture\n\r\n\r");
//        return 1;
//    }
//    return 0;
//}

/**
 * @brief get gpio pin polarity level when interrupt occure
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] *logic_level poin to the polarity level that reflect on the pin when there is an interrupt
 * @return status code
 *          - 0 success
 *          - 1 failed get pin polarity level
 * @note    none
 */
uint8_t mcp23008_get_pin_interrupt_caputure_level(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_captured_state_t *logic_level) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_CAPTURED_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read interrupt capture status");
        return 1;
    }
    //    status &= pinMask[pin];
    *logic_level = (mcp23008_interrupt_captured_state_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set interrupt pin output mode (open drain or active mode)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] mode is the outout mode
 * @return status code
 *          - 0 success
 *          - 1 failed set interrupt putput mode
 * @note    none
 */
uint8_t mcp23008_set_interrupt_pin_output_mode(mcp23008_handle_t *const pHandle, mcp23008_interrupt_open_drain_mode_t mode) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read expander reg");
        return 1;
    }
    status &= ~(1 << 2);
    status |= (mode << 2);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set interrupt pin output mode");
        return 1;
    }
    return 0;
}

/**
 * @brief Get interrupt pin output mode (open drain or not)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[out] *mode is the outout mode
 * @return status code
 *          - 0 success
 *          - 1 failed get interrupt putput mode
 * @note    none
 */
uint8_t mcp23008_get_interrupt_pin_output_mode(mcp23008_handle_t *const pHandle, mcp23008_interrupt_open_drain_mode_t *mode) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read pin interrupt status");
        return 1;
    }
    *mode = (mcp23008_interrupt_open_drain_mode_t) ((status & MCP23008_ODR_MASK) >> 2);
    return 0;
}

/**
 * @brief set the mode on how the associated pin value is compared for the interrupt-on-change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] mode is the compare value mode
 * @return status code
 *          - 0 success
 *          - 1 failed set interrupt compare mode
 * @note    none
 */
uint8_t mcp23008_set_ineterrupt_compare_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_compare_value_t mode) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_ON_CHANGE_CONTROL_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read interrupt on change control reg");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (mode << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_INTERRUPT_ON_CHANGE_CONTROL_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"set interrupt compare mode");
        return 1;
    }
    return 0;
}

/**
 * @brief get the mode on how the associated pin value is compared for the interrupt-on-change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] *mode point the compare value mode
 * @return status code
 *          - 0 success
 *          - 1 failed get interrupt compare mode
 * @note    none
 */
uint8_t mcp23008_get_ineterrupt_compare_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_compare_value_t *mode) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_INTERRUPT_ON_CHANGE_CONTROL_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read interrupt capture status");
        return 1;
    }
    status &= pinMask[pin];
    *mode = (mcp23008_interrupt_compare_value_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set the compare value for pins configured for interrupt-on-change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] value is the logic level
 * @return status code
 *          - 0 success
 *          - 1 failed to set interrupt default value
 * @note    none
 */
uint8_t mcp23008_set_default_compare_value(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_default_value_t value) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_DEFAULT_VALUE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read default compare value reg");
        return 1;
    }
    status &= ~(1 << pin);
    status |= (value << pin);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_DEFAULT_VALUE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set pin default compare value");
        return 1;
    }
    return 0;

}

/**
 * @brief get the compare value for pins configured for interrupt-on-change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] *value is the logic level
 * @return status code
 *          - 0 success
 *          - 1 failed to get interrupt default value
 * @note    none
 */
uint8_t mcp23008_get_default_compare_value(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_default_value_t *value) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL)      /**< check handle */
        return 2;            /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3;            /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_DEFAULT_VALUE_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read pin default compare value status");
        return 1;
    }
    status &= pinMask[pin];
    *value = (mcp23008_interrupt_default_value_t) ((status & pinMask[pin]) >> pin);
    return 0;
}

/**
 * @brief set Slew Rate Control Bit for SDA Output
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] enable is the boolean logic level to set
 * @return status code
 *          - 0 success
 *          - 1 failed set slew rate
 * @note    none
 */
uint8_t mcp23008_set_slew_rate(mcp23008_handle_t *const pHandle, mcp23008_bool_t enable) {
    uint8_t err;
    uint8_t status;

    enable = !enable;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read expander reg");
        return 1;
    }
    status &= ~(1 << 4);
    status |= (enable << 4);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set slew rate");
        return 1;
    }
    return 0;
}

/**
 * @brief get Slew Rate Control Bit for SDA Output
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] *enable point to the the boolean logic level set
 * @return status code
 *          - 0 success
 *          - 1 failed get slew rate
 * @note    none
 */
uint8_t mcp23008_get_slew_rate(mcp23008_handle_t *const pHandle, mcp23008_bool_t *enable) {
    uint8_t err;
    uint8_t status;


    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
       a_mcp23008_print_error_msg(pHandle, "read slew rate status");
        return 1;
    }
    *enable = (mcp23008_bool_t) ((status & MCP23008_DISSLW_MASK) >> 4);
    *enable = ! *enable;
    return 0;
}

/**
 * @brief set Sequential Operation Mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] enable is the boolean logic level to set
 * @return status code
 *          - 0 success
 *          - 1 failed to set sequential operation mode
 * @note    none
 */
uint8_t mcp23008_set_sequencial_mode(mcp23008_handle_t *const pHandle, mcp23008_bool_t enable) {
    uint8_t err;
    uint8_t status;

    enable = !enable;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "read expander reg");
        return 1;
    }
    status &= ~(1 << 5);
    status |= (enable << 5);
    err = a_mcp23008_i2c_write(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle, "set sequential operation mode");
        return 1;
    }
    return 0;
}

/**
 * @brief get Sequential Operation Mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] *enable point to the boolean logic level set
 * @return status code
 *          - 0 success
 *          - 1 failed get sequential operation mode
 * @note    none
 */
uint8_t mcp23008_get_sequencial_mode(mcp23008_handle_t *const pHandle, mcp23008_bool_t *enable) {
    uint8_t err;
    uint8_t status;

    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    err = a_mcp23008_i2c_read(pHandle, MCP23008_EXPANDER_CONFIGURATION_REG, (uint8_t *) &status, 1);
    if (err != 0) {
        a_mcp23008_print_error_msg(pHandle,"read sequencial operation mode status");
        return 1;
    }
    *enable = (mcp23008_bool_t) ((status & MCP23008_SEQOP_MASK) >> 5);
    *enable = ! *enable;
    return 0;
}

/**
 * @brief     set the chip register
 * @param[in] *handle points to mcp23008 handle structure
 * @param[in] reg is the i2c register address
 * @param[in] *buf points to a data buffer
 * @param[in] length is the data buffer length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */

uint8_t mcp23008_set_reg(mcp23008_handle_t *const pHandle, uint8_t reg, uint8_t *buf, uint16_t length) {
    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    if (a_mcp23008_i2c_write(pHandle->i2c_address, reg, buf, length) != 0) {
        return 1;
    }
    return 0;

}

/**
 * @brief      get the chip register
 * @param[in]  *handle points to mcp23008 handle structure
 * @param[in]  reg is the i2c register address
 * @param[out] *buf points to a data buffer
 * @param[in]  length is the data buffer length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 *             - 2 handle is NULL
 *             - 3 handle is not initialized
 * @note       none
 */
uint8_t mcp23008_get_reg(mcp23008_handle_t *const pHandle, uint8_t reg, uint8_t *buf, uint16_t length) {
    if (pHandle == NULL) /**< check handle */
        return 2; /**< return error */
    if (pHandle->inited != 1) /**< check initialize status */
        return 3; /**< return error */

    if (a_mcp23008_i2c_read(pHandle->i2c_address, reg, buf, length) != 0) {
        return 1;
    }
    return 0;
}

/**
 * @brief      get chip's information
 * @param[out] *info points to mcp23008 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mcp23008_info(mcp23008_handle_t *const pHandle) {

    strncpy(pHandle->info.chip_name, CHIP_NAME, 10); /**< copy chip name */
    strncpy(pHandle->info.interface, INTERFACE, 5); /**< copy interface name */
    strncpy(pHandle->info.manufacturer_name, MANUFACTURER_NAME, 25); /**< copy manufacturer name */
    pHandle->info.supply_voltage_max_v = SUPPLY_VOLTAGE_MAX; /**< set minimal supply voltage */
    pHandle->info.supply_voltage_min_v = SUPPLY_VOLTAGE_MIN; /**< set maximum supply voltage */
    pHandle->info.max_current_ma = MAX_CURRENT; /**< set maximum current */
    pHandle->info.temperature_max = TEMPERATURE_MAX; /**< set minimal temperature */
    pHandle->info.temperature_min = TEMPERATURE_MIN; /**< set maximum temperature */
    pHandle->info.driver_version = DRIVER_VERSION; /**< set driver version */

    return 0; /**< success */
}

/*end*/
