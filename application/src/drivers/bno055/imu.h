/** ================================================================
 * @file    application/src/drivers/bno055/imu.c
 *
 * @brief   imu.h define low level functions to control the IMU sensor.
 *
 * @date    25-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

// Include once
#ifndef DEF_IMU
#define DEF_IMU

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Libs
#include "../../init/init.h"

// BNO lib
#include "bno055.h"

/* -----------------------------------------------------------------
 * FETCHING NODE PARAMETERS
 * -----------------------------------------------------------------
 */

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND THE IMU
 * -----------------------------------------------------------------
 */

/**
 * @brief   This function perform a read on the target device.
 *
 * @param   dev_addr    I2C device address. Since we're using UART, this parameter is ignored.
 * @param   reg_addr    read register address
 * @param   reg_data    pointer to an empty data target to be filled
 * @param   r_len       number of registers to be read-out.
 *
 * @return  status code
 * @retval  0 (BNO_SUCESS) : OK
 * @retval  1 (BNO_FAIL) : NOK
 */
s8 bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len);

/**
 * @brief   This function perform a write on the target device
 *
 * @param   dev_addr    I2C device address. Since we're using UART, this parameter is ignored.
 * @param   reg_addr    read register address
 * @param   reg_data    pointer to the data to be written
 * @param   r_len       number of registers to be wrote.
 *
 * @return  status code
 * @retval  0 (BNO_SUCESS) : OK
 * @retval  1 (BNO_FAIL) : NOK
 */
s8 bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 wr_len);

/**
 * @brief   This function delay the system.
 *
 * @warning Due to the usage of zepyr, this function is actually an alias to the k_msleep function.
 *
 * @param   delay_in_msec The value in msec to be delayed.
 *
 * @return  Nothing
 */
void delay_func(int delay_in_msec);

#endif /* DEF_IMU*/
