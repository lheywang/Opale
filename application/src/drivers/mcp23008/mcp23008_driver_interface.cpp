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
 * File:   mcp23008_driver_interface.c
 * Author: Cedric Akilimali
 *
 * Created on August 04, 2022, 1:25 PM
 */
#include "mcp23008_driver_interface.h"
#include "mcp23008_driver_basic.h"

/**
 * @brief  interface i2c bus init
 * @return status code
 *         - 0 success
 *         - 1 i2c init failed
 * @note   none
 */
uint8_t mcp23008_interface_i2c_init(void)
{
    /*call your i2c initialize function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

/**
 * @brief interface i2c bus deinit
 * @return status code
 *          - 0 success
 *          - 1 i2c deinit fail
 */
uint8_t mcp23008_interface_i2c_deinit(void)
{
    /*call your i2c de-initialize function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

/**
 * @brief      interface i2c bus read
 * @param[in]  addr is the i2c device write address
 * @param[in]  reg is the i2c register address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mcp23008_interface_i2c_read(uint8_t u8Addr, uint8_t *pBuf, uint16_t u8Length)
{
    /*call your i2c read function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

/**
 * @brief     interface i2c bus write
 * @param[in] addr is the i2c device write address
 * @param[in] reg is the i2c register address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mcp23008_interface_i2c_write(uint8_t u8Addr, uint8_t *pBuf, uint16_t u8Length)
{
    /*call your i2c write function here*/
    /*user code begin */

    /*user code end*/
    return 0; /**< success */
}

/**
 * @brief     interface delay ms
 * @param[in] U32Ms
 * @note      none
 */
void mcp23008_interface_delay_ms(uint32_t U32Ms)
{
    /*call your delay function here*/
    /*user code begin */

    /*user code end*/
}

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void mcp23008_interface_debug_print(char *const fmt, ...)
{
    /*call your call print function here*/
    /*user code begin */
#ifdef MCP23008_DEBUG_MODE
    volatile char str[256];
    volatile uint8_t len;
    va_list args;

    memset((char *)str, 0, sizeof(char) * 256);
    va_start(args, fmt);
    vsnprintf((char *)str, 256, (char const *)fmt, args);
    va_end(args);

    len = strlen((char *)str);
    //    EUSART1_Write_Text((const char *) str, len);
    (void)printf((char *)str, len);

    /*user code end*/
#endif
}

/**
 * @brief     interface receive callback
 * @param[in] type is the interrupt type
 * @note      none
 */
void mcp23008_interface_receive_callback(uint8_t type)
{
    /*call your interrupt callback function here*/
    /*user code begin */
    switch (type)
    {
    case MCP23008_GPIO_PIN_0:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP0 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_1:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP1 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_2:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP2 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_3:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP3 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_4:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP4 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_5:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP5 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_6:
    {

        //            mcp23008_interface_debug_print("mcp23008: GP6 interrupt\n");
        break;
    }

    case MCP23008_GPIO_PIN_7:
    {
        //            mcp23008_interface_debug_print("mcp23008: GP7 interrupt\n");
        break;
    }

    default:
        mcp23008_interface_debug_print("mcp23008:false interrupt t\n");
        break;
    }

    /*user code end*/
}

/*end*/
