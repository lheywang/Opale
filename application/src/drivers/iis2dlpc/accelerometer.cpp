/** ================================================================
 * @file    application/src/devices/accelerometer.c
 *
 * @brief   accelerometer.c implement some low level functions for the
 *          control of the accelerometer sensor.
 *
 * @date    25-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Libs
#include "init/init.hpp"

// Local lib
#include "drivers/iis2dlpc_reg.h"

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND AN ACCELEROMETER
 * -----------------------------------------------------------------
 */

int32_t iis2dlpc_write_reg(const stmdev_ctx_t *ctx,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t len)
{
    // TODO Implement here !
    return 0;
}

int32_t iis2dlpc_read_reg(const stmdev_ctx_t *ctx,
                          uint8_t reg,
                          uint8_t *data,
                          uint16_t len)
{
    // TODO Implement here !
    return 0;
}
