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
 * File:   mcp23008_driver_interface.h
 * Author: Cedric Akilimali
 *
 * Created on August 04, 2022, 1:25 PM
 */

#ifndef MCP23008_DRIVER_INTERFACE_H_INCLUDED
#define MCP23008_DRIVER_INTERFACE_H_INCLUDED

#include "mcp23008_driver.h"

static mcp23008_handle_t mcp23008_handle;

/**
 * @brief  interface i2c bus init
 * @return status code
 *         - 0 success
 *         - 1 i2c init failed
 * @note   none
 */
uint8_t mcp23008_interface_i2c_init(void);

/**
 * @brief interface i2c bus deinit
 * @return status code
 *          - 0 success
 *          - 1 i2c deinit fail
 */
uint8_t mcp23008_interface_i2c_deinit(void);

/**
 * @brief      interface i2c bus read
 * @param[in]  addr is the i2c device write address
 * @param[out] *buf points to a data buffer
 * @param[in]  len is the length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mcp23008_interface_i2c_read(uint8_t u8Addr, uint8_t *pBuf, uint16_t u8Length);

/**
 * @brief     interface i2c bus write
 * @param[in] addr is the i2c device write address
 * @param[in] *buf points to a data buffer
 * @param[in] len is the length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mcp23008_interface_i2c_write(uint8_t u8Addr, uint8_t *pBuf, uint16_t u8Length);

/**
 * @brief     interface delay ms
 * @param[in] U32Ms
 * @note      none
 */
void mcp23008_interface_delay_ms(uint32_t U32Ms);

/**
 * @brief     interface print format data
 * @param[in] fmt is the format data
 * @note      none
 */
void mcp23008_interface_debug_print(char *const fmt, ...);

/**
 * @brief     interface receive callback
 * @param[in] type is the interrupt type
 * @note      none
 */
void mcp23008_interface_receive_callback(uint8_t type);

#endif // MCP23008_DRIVER_INTERFACE_H_INCLUDED
