/** ================================================================
 * @file    application/src/devices/accelerometer.h
 *
 * @brief   accelerometer.h define low level functions to control
 *          the accelerometer I2C sensor (optionnal on the board).
 *
 * @date   25-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

// Include once
#ifndef DEF_ACCELEROMETER
#define DEF_ACCELEROMETER

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Libs
#include "init/init.hpp"

// Local lib
#include "drivers/iis2dlpc_reg.h"

/* -----------------------------------------------------------------
 * FETCHING NODE PARAMETERS
 * -----------------------------------------------------------------
 */

/* -----------------------------------------------------------------
 * FUNCTIONS THAT OVERHIDE THE DEFAULT ONES
 * -----------------------------------------------------------------
 */

/**
 * @brief   This function overhide another one, defined into the iis2dlpc_reg.h file.
 *          This is used to read to a register over our proper bus system.
 *
 * @param   ctx     This is a pointer to a stmdev struct, and remains unused in our case.
 * @param   reg     This represent the register address where we're going to read.
 * @param   data    This represent a pointer to the data which is going to be rode.
 * @param   len     This reprensent the number of bytes we're going to read.
 *
 * @retval   0      No error
 * @retval  -1      Error for ...
 *
 */
int32_t iis2dlpc_read_reg(const stmdev_ctx_t *ctx,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t len);

/**
 * @brief   This function overhide another one, defined into the iis2dlpc_reg.h file.
 *          This is used to write to a register over our proper bus system.
 *
 * @param   ctx     This is a pointer to a stmdev struct, and remains unused in our case.
 * @param   reg     This represent the register address where we're going to write.
 * @param   data    This represent a pointer to the data which is going to be wrote.
 * @param   len     This reprensent the number of bytes we're going to write.
 *
 * @retval   0      No error
 * @retval  -1      Error for ...
 *
 */
int32_t iis2dlpc_write_reg(const stmdev_ctx_t *ctx,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t len);

#endif /*DEF_ACCELEROMETER */
