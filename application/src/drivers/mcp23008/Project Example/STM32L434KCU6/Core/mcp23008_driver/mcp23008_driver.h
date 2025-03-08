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
 * File:   mcp23008_driver.h
 * Author: Cedric Akilimali
 *
 * Created on August 02, 2022, 3:13 PM
 */

#ifndef MCP23008_DRIVER_H_INCLUDED
#define MCP23008_DRIVER_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>

#define MCP23008_DEBUG_MODE

/**
 * @defgroup driver_mcp23008 mcp23008 driver function
 * @brief mcp23008 driver modules
 * @{
 */

/**
 * @addtogrouup mcp23008_base_driver
 * @{
 */

/*Control Register */

#define MCP23008_DIRECTION_REG                      0x00       /**< I/O Direction Register */
#define MCP23008_INPUT_POLARITY_REG                 0x01       /**< Input Polarity port Register */
#define MCP23008_INTERRUPT_ON_CHANGE_REG            0x02       /**< Interrupt on Change Pins register */
#define MCP23008_DEFAULT_VALUE_REG                  0x03       /**< Default Value Register */
#define MCP23008_INTERRUPT_ON_CHANGE_CONTROL_REG    0x04       /**< Interrupt on Change control register */
#define MCP23008_EXPANDER_CONFIGURATION_REG         0x05       /**< I/O Expander Configuration Register */
#define MCP23008_GPIO_PULLUP_REG                    0x06       /**< GPIO Pull-up Resistor register */
#define MCP23008_INTERRUPT_FLAG_REG                 0x07       /**< Interrupt Flag Register */
#define MCP23008_INTERRUPT_CAPTURED_REG             0x08       /**< Interrupt Captured Value for Port Register */
#define MCP23008_GENERAL_PURPOSE_REG                0x09       /**< General Purpose I/O Port Register */
#define MCP23008_OUTPUT_LATCH_REG                   0x0A       /**< Output Latch Register */

/*Register Mask */

#define MCP23008_SEQOP_MASK                         0x20        /**< Sequential Operation Mode Mask */
#define MCP23008_DISSLW_MASK                        0x10        /**< Slew Rate Control Bit for SDA Output Mask */
#define MCP23008_HAEN_MASK                          0x08        /**< Hardware Address Enable mask */
#define MCP23008_ODR_MASK                           0x04        /**< INT pin open-drain output Mask*/
#define MCP23008_INTPOL_MASK                        0x02        /**< Interrupt polarity Mask */

/* GPIO Pin Mask */

#define MCP23008_GP0_MASK                           0x01        /**< GPIO pin 0 Mask */
#define MCP23008_GP1_MASK                           0x02        /**< GPIO pin 1 Mask */
#define MCP23008_GP2_MASK                           0x04        /**< GPIO pin 2 Mask */
#define MCP23008_GP3_MASK                           0x08        /**< GPIO pin 3 Mask */
#define MCP23008_GP4_MASK                           0x10        /**< GPIO pin 4 Mask */
#define MCP23008_GP5_MASK                           0x20        /**< GPIO pin 5 Mask */
#define MCP23008_GP6_MASK                           0x40        /**< GPIO pin 6 Mask */
#define MCP23008_GP7_MASK                           0x80        /**< GPIO pin 7 Mask */

#define MCP23008_MAX_NUM_GPIO_PIN                   0x08

#define HIGH                                        0x01
#define LOW                                         0x00

static uint8_t err;

static uint8_t pinMask[MCP23008_MAX_NUM_GPIO_PIN] = {MCP23008_GP0_MASK,
                                                    MCP23008_GP1_MASK,
                                                    MCP23008_GP2_MASK,
                                                    MCP23008_GP3_MASK,
                                                    MCP23008_GP4_MASK,
                                                    MCP23008_GP5_MASK,
                                                    MCP23008_GP6_MASK,
                                                    MCP23008_GP7_MASK};

/**
* @brief mcp794xx callback data definition
*/
typedef uint8_t(*mcp23008_irq_callback_t)(void);

/**
 * @brief mcp23008 i2c address enumeration
 */
typedef enum {
    MCP23008_ADDRESS_DEFAULT  = 0x20, /**<  7bit Default slave Address */
    MCP23008_I2C_ADDRESS_PIN_A000 = 0x00, /**<  A2A1A0 000 */
    MCP23008_I2C_ADDRESS_PIN_A001 = 0x01, /**<  A2A1A0 001 */
    MCP23008_I2C_ADDRESS_PIN_A010 = 0x02, /**<  A2A1A0 010 */
    MCP23008_I2C_ADDRESS_PIN_A011 = 0x03, /**<  A2A1A0 011 */
    MCP23008_I2C_ADDRESS_PIN_A100 = 0x04, /**<  A2A1A0 100 */
    MCP23008_I2C_ADDRESS_PIN_A101 = 0x05, /**<  A2A1A0 101 */
    MCP23008_I2C_ADDRESS_PIN_A110 = 0x06, /**<  A2A1A0 110 */
    MCP23008_I2C_ADDRESS_PIN_A111 = 0x07, /**<  A2A1A0 111 */
} mcp23008_address_pin_t;

/**
 * @brief mcp23008 boolean enumeration
 */
typedef enum {
    MCP23008_BOOL_FALSE = 0x00, /**< Boolean state false */
    MCP23008_BOOL_TRUE  = 0x01 /**< Boolean state true */
} mcp23008_bool_t;

/**
 * @brief mcp23008 port mode enumeration
 */
typedef enum {
    MCP23008_OUTPUT = 0x00, /**< Port configured as output mode */
    MCP23008_INPUT  = 0x01, /**< Port configured as input mode */
    MCP23008_INPUT_PULLUP = 0x10 /**< Port configured as input pull-up mode */
} mcp23008_port_mode_t;

/**
 * @brief mcp23008 gpio port enumeration
 */
typedef enum {
    MCP23008_GPIO_PIN_0 = 0x00, /**< gpio port GP0 */
    MCP23008_GPIO_PIN_1 = 0x01, /**< gpio port GP1 */
    MCP23008_GPIO_PIN_2 = 0x02, /**< gpio port GP2 */
    MCP23008_GPIO_PIN_3 = 0x03, /**< gpio port GP3 */
    MCP23008_GPIO_PIN_4 = 0x04, /**< gpio port GP4 */
    MCP23008_GPIO_PIN_5 = 0x05, /**< gpio port GP5 */
    MCP23008_GPIO_PIN_6 = 0x06, /**< gpio port GP6 */
    MCP23008_GPIO_PIN_7 = 0x07  /**< gpio port GP7 */
} mcp23008_gpio_port_t;

/**
 * @brief mcp23008 port polarity enumeration
 */
typedef enum {
    MCP23008_POLARITY_SAME_LOGIC_STATE = 0x00, /**< GPIO register bit will reflect the same logic state of the input pin */
    MCP23008_POLARITY_OPPOSITE_LOGIC_STATE = 0x01 /**< GPIO register bit will reflect the opposite logic state of the input pin */
} mcp23008_port_polarity_t;

/**
 * @brief mcp23008 interrupt flag enumeration
 */
typedef enum {
    MCP23008_interrupt_CLEAR = 0x00, /**< Interrupt flag cleared */
    MCP23008_interrupt_SET = 0x01 /**< Interrupt flag set */
} mcp23008_interrupt_flag_t;

/**
 * @brief mcp23008 interrupt compare value enumeration
 */
typedef enum {
    MCP23008_interrupt_COMP_TO_PREVIOUS_VALUE = 0x00, /**< Interrupt on change compares to value set on default register */
    MCP23008_interrupt_COMP_TO_DEFAULT_VALUE = 0x01 /**< Interrupt on change compares to value previous port value */
} mcp23008_interrupt_compare_value_t;

/**
 * @brief mcp23008 interrupt default value enumeration
 */
typedef enum {
    MCP23008_interrupt_RISING_EDGE  = 0x00, /**< Interrupt on change default value to compare 0 */
    MCP23008_interrupt_FALLING_EDGE = 0x01  /**< Interrupt on change default value to compare 1 */
} mcp23008_interrupt_default_value_t;

/**
 * @brief mcp23008 interrupt open-drain enumeration
 */
typedef enum {
    MCP23008_interrupt_ACTIVE_DRIVER = 0x00, /**< Active driver output (INTPOL bit sets the polarity) */
    MCP23008_interrupt_OPEN_DRAIN_OUTPUT = 0x01 /**< Open-drain output (overrides the INTPOL bit) */
} mcp23008_interrupt_open_drain_mode_t;

/**
 * @brief mcp23008 interrupt polarity enumeration
 */
typedef enum {
    MCP23008_interrupt_ACTIVE_LOW = 0x00, /**< Interrupt active low */
    MCP23008_interrupt_ACTIVE_HIGH = 0x01 /**< Interrupt active high */
} mcp23008_interrupt_polarity_t;

/**
 * @brief mcp23008 interrupt captured value enumeration
 */
typedef enum {
    MCP23008_interrupt_CAP_LOW = 0x00, /**< GPIO interrupt captured logic low */
    MCP23008_interrupt_CAP_HIGH = 0x01 /**< GPIO interrupt captured logic high */
} mcp23008_interrupt_captured_state_t;

/**
 * @brief mcp23008 logic level enumeration
 */
typedef enum {
    MCP23008_GPIO_LOW = 0x00, /**< GPIO port logic level low */
    MCP23008_GPIO_HIGH = 0x01 /**< GPIO port logic level high */
} mcp23008_port_logic_level_t;

/**
 * @brief mcp23008 information structure definition
 */
typedef struct mcp23008_info_s {
    char chip_name[10]; /**< chip name 1 */
    char manufacturer_name[25]; /**< manufacturer name */
    char interface[5]; /**< chip interface name */
    float supply_voltage_min_v; /**< chip min supply voltage */
    float supply_voltage_max_v; /**< chip max supply voltage */
    float max_current_ma; /**< chip max current */
    float temperature_min; /**< chip min operating temperature */
    float temperature_max; /**< chip max operating temperature */
    float driver_version; /**< driver version */
} mcp23008_info_t;

/**
 * @brief mcp23008 handle structure definition
 */
typedef struct mcp23008_handle_s {

    uint8_t(*i2c_init)(void); /**< point to a iic init function address */
    uint8_t(*i2c_deinit)(void); /**< point to a iic deinit function address */
    uint8_t(*i2c_write)(uint8_t addr, uint8_t *buf, uint16_t len); /**< point to a i2c write function address */
    uint8_t(*i2c_read)(uint8_t addr, uint8_t *buf, uint16_t len); /**< point to a i2c read function address */
    void (*delay_ms)(uint32_t ms); /**< point to a delay_ms function address */
    void(*debug_print)(char *fmt, ...); /**< point to a debug_print function address */
    void (*receive_callback)(uint8_t type); /**< point to a receive callback function address */
    mcp23008_info_t info;
    uint8_t i2c_address; /**< i2c device address */
    uint8_t inited; /**< inited flag */

} mcp23008_handle_t;


/**
 * @}
 */

/**
 * @defgroup mcp23008_link_driver mcp23008 link driver function
 * @brief    mcp23008 link driver modules
 * @ingroup  mcp23008 driver
 * @{
 */

/**
 * @brief     initialize mcp23008_handle_t structure
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] STRUCTURE is mcp23008_handle_t
 * @note      none
 */
#define DRIVER_MCP23008_LINK_INIT(HANDLE, STRUCTURE)           memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 * @brief     link i2c_init function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a i2c_init function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_I2C_INIT(HANDLE, FUC)              (HANDLE)->i2c_init = FUC


/**
 * @brief     link i2c_deinit function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a i2c_deinit function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_I2C_DEINIT(HANDLE, FUC)            (HANDLE)->i2c_deinit = FUC


/**
 * @brief     link i2c_write function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a i2c_write function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_I2C_WRITE(HANDLE, FUC)             (HANDLE)->i2c_write = FUC


/**
 * @brief     link i2c_read function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a i2c_read function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_I2C_READ(HANDLE, FUC)              (HANDLE)->i2c_read = FUC

/**
 * @brief     link delay_ms function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a delay_ms function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_DELAY_MS(HANDLE, FUC)             (HANDLE)->delay_ms = FUC

/**
 * @brief     link debug_print function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a debug_print function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_DEBUG_PRINT(HANDLE, FUC)          (HANDLE)->debug_print = FUC

/**
 * @brief     link receive_callback function
 * @param[in] HANDLE points to mcp23008 handle structure
 * @param[in] FUC points to a receive_callback function address
 * @note      none
 */
#define DRIVER_MCP23008_LINK_RECEIVE_CALLBACK(HANDLE, FUC)     (HANDLE)->receive_callback = FUC


/**
 * @}
 */

/**
 * @defgroup mcp23008_base_driver mcp23008 base driver function
 * @brief    mcp23008 base driver modules
 * @ingroup  mcp23008_driver
 * @{
 */

/**
 * @brief      get chip's information
 * @param[out] *info points to mcp23008 info structure
 * @return     status code
 *             - 0 success
 *             - 2 handle is NULL
 * @note       none
 */
uint8_t mcp23008_info(mcp23008_handle_t *const pHandle);

/**
 * @brief     initialize the chip
 * @param[in] *handle points to mcp23008 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 i2c initialization failed
 *            - 2 handle is NULL
 *            - 3 linked functions is NULL
 * @note      none
 */
uint8_t mcp23008_init(mcp23008_handle_t *const pHandle);

/**
 * @brief     close the chip
 * @param[in] *handle points to mcp23008 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 i2c deinit failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mcp23008_deinit(mcp23008_handle_t *const pHandle);

/**
 * @brief     irq handler
 * @param[in] *handle points to mcp23008 handle structure
 * @return    status code
 *            - 0 success
 *            - 1 run failed
 *            - 2 handle is NULL
 *            - 3 handle is not initialized
 * @note      none
 */
uint8_t mcp23008_irq_handler(mcp23008_handle_t *const pHandle);

/**
 * @brief     set the address pin
 * @param[in] *handle points to a mcp23008 handle structure
 * @param[in] address_pin is the chip address pins
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mcp23008_set_addr_pin(mcp23008_handle_t *const pHandle, mcp23008_address_pin_t address_pin);

/**
 * @brief     get the address pin
 * @param[in] *handle points to a mcp23008 handle structure
 * @param[in] *addr_pin point to the chip address pins
 * @return    status code
 *            - 0 success
 *            - 2 handle is NULL
 * @note      none
 */
uint8_t mcp23008_get_addr_pin(mcp23008_handle_t *const pHandle, mcp23008_address_pin_t *addr_pin);

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
uint8_t mcp23008_set_pin_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_mode_t mode);

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
uint8_t mcp23008_get_pin_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_mode_t *mode);

/**
 * @brief set port mode all pins to either (input, output or input_pullup)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] mode is the mode gpio port to set
 * @return status code
 *          - 0 success
 *          - 1 failed to set pin mode
 * @note   none
 */
uint8_t mcp23008_set_port_mode(mcp23008_handle_t *const pHandle, mcp23008_port_mode_t mode);

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
uint8_t mcp23008_pin_write(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_logic_level_t logic_level);

/**
 * @brief Write logic value to all gpio pins
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] logic_level is the logic level to assign to the pin
 * @return status code
 *          - 0 success
 *          - 1 failed to write pin
 * @note    none
 */
uint8_t mcp23008_pin_write_all(mcp23008_handle_t *const pHandle, mcp23008_port_logic_level_t logic_level);

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
uint8_t mcp23008_pin_read(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_logic_level_t *logic_level);

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
uint8_t mcp23008_set_pin_pullup_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t enable);

/**
 * @brief set port pull-up mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] enable is the boolean logic to set
 * @return status code
 *          - 0 success
 *          - 1 failed failed to set pull-up mode
 * @note    none
 */
uint8_t mcp23008_set_port_pullup_mode(mcp23008_handle_t *const pHandle, mcp23008_bool_t enable);

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
uint8_t mcp23008_get_pin_pullup_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t *enable);

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
uint8_t mcp23008_set_pin_input_polarity_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_polarity_t polarity);

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
uint8_t mcp23008_get_pin_input_polarity_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_port_polarity_t *polarity);


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
uint8_t mcp23008_set_pin_interrupt(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t enable);

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
uint8_t mcp23008_get_pin_interrupt(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_bool_t *enable);

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
uint8_t mcp23008_get_interrupt_flag(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_flag_t *flag);

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
uint8_t mcp23008_clear_interrupt_flag(mcp23008_handle_t *const pHandle/*, mcp23008_gpio_port_t pin, mcp23008_interrupt_flag_t flag*/);

/**
 * @brief set interrupt output pin level when an interrupt occur
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] polarity is the interrupt pin level
 * @return status code
 *          - 0 success
 *          - 1 failed to set interrupt logic output logic level
 * @note    none
 */
uint8_t mcp23008_set_intrrupt_pin_output_level(mcp23008_handle_t *const pHandle, mcp23008_interrupt_polarity_t logic_level);

/**
 * @brief get interrupt output pin level assigned when an interrupt occur
 * @param[in] *handle point to mcp23008 handle structure
 * @param[out] *polarity is the interrupt pin level
 * @return status code
 *          - 0 success
 *          - 1 failed to get interrupt logic output logic level
 * @note    none
 */
uint8_t mcp23008_get_interrupt_pin_output_level(mcp23008_handle_t *const pHandle, mcp23008_interrupt_polarity_t *polarity);

/**
 * @brief set gpio pin polarity level when interrupt occur (
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] logic_level is the polarity level that will reflect on the pin when there is an interrupt
 * @return status code
 *          - 0 success
 *          - 1 failed set pin polarity level
 * @note    none
 */
uint8_t mcp23008_set_pin_interrupt_caputure_level(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_captured_state_t logic_level);

/**
 * @brief get gpio pin polarity level when interrupt occur
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] *logic_level point to the polarity level that reflect on the pin when there is an interrupt
 * @return status code
 *          - 0 success
 *          - 1 failed set pin polarity level
 * @note    none
 */
uint8_t mcp23008_get_pin_interrupt_caputure_level(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_captured_state_t *logic_level);

/**
 * @brief set interrupt pin output mode (open drain or active mode)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] mode is the output mode
 * @return status code
 *          - 0 success
 *          - 1 failed set interrupt output mode
 * @note    none
 */
uint8_t mcp23008_set_interrupt_pin_output_mode(mcp23008_handle_t *const pHandle, mcp23008_interrupt_open_drain_mode_t mode);

/**
 * @brief Get interrupt pin output mode (open drain or not)
 * @param[in] *handle point to mcp23008 handle structure
 * @param[out] *mode is the output mode
 * @return status code
 *          - 0 success
 *          - 1 failed get interrupt output mode
 * @note    none
 */
uint8_t mcp23008_get_interrupt_pin_output_mode(mcp23008_handle_t *const pHandle, mcp23008_interrupt_open_drain_mode_t *mode);

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
uint8_t mcp23008_set_ineterrupt_compare_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_compare_value_t mode);

/**
 * @brief get the mode on how the associated pin value is compared for the interrupt-on-change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] *mode point the compare value mode
 * @return status code
 *          - 0 success
 *          - 1 failed set interrupt compare mode
 * @note    none
 */
uint8_t mcp23008_get_ineterrupt_compare_mode(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_compare_value_t *mode);

/**
 * @brief set the compare value of pins configured as interrupt-on-change
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] pin is the gpio pin to configurer
 * @param[in] value is the logic level
 * @return status code
 *          - 0 success
 *          - 1 failed to set interrupt default value
 * @note    none
 */
uint8_t mcp23008_set_default_compare_value(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_default_value_t value);

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
uint8_t mcp23008_get_default_compare_value(mcp23008_handle_t *const pHandle, mcp23008_gpio_port_t pin, mcp23008_interrupt_default_value_t *value);

/**
 * @brief set Slew Rate Control Bit for SDA Output
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] enable is the boolean logic level to set
 * @return status code
 *          - 0 success
 *          - 1 failed set slew rate
 * @note    none
 */
uint8_t mcp23008_set_slew_rate(mcp23008_handle_t *const pHandle, mcp23008_bool_t enable);

/**
 * @brief get Slew Rate Control Bit for SDA Output
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] *enable point to the the boolean logic level set
 * @return status code
 *          - 0 success
 *          - 1 failed get slew rate
 * @note    none
 */
uint8_t mcp23008_get_slew_rate(mcp23008_handle_t *const pHandle, mcp23008_bool_t *enable);

/**
 * @brief set Sequential Operation Mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] enable is the boolean logic level to set
 * @return status code
 *          - 0 success
 *          - 1 failed set sequential operation mode
 * @note    none
 */
uint8_t mcp23008_set_sequencial_mode(mcp23008_handle_t *const pHandle, mcp23008_bool_t enable);

/**
 * @brief get Sequential Operation Mode
 * @param[in] *handle point to mcp23008 handle structure
 * @param[in] *enable point to the boolean logic level set
 * @return status code
 *          - 0 success
 *          - 1 failed get sequential operation mode
 * @note    none
 */
uint8_t mcp23008_get_sequencial_mode(mcp23008_handle_t *const pHandle, mcp23008_bool_t *enable);

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

uint8_t mcp23008_set_reg(mcp23008_handle_t *const pHandle, uint8_t reg, uint8_t *buf, uint16_t length);

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
uint8_t mcp23008_get_reg(mcp23008_handle_t *const pHandle, uint8_t reg, uint8_t *buf, uint16_t length);

/**
 * @}
 */

/**
 * @}
 */

#endif /**< MCP23008_DRIVER_H_INCLUDED*/
