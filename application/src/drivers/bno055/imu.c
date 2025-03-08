/** ================================================================
 * @file    application/src/drivers/bno055/imu.c
 *
 * @brief   imu.c implement some low level functions for the
 *          control of the imu sensor.
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
#include "../../init/init.h"

// Zephyr
#include <zephyr/kernel.h>

// BNO lib
#include "bno055.h"

/* -----------------------------------------------------------------
* FUNCTIONS TO COMMAND AN IMU SENSOR
* -----------------------------------------------------------------
*/

s8 bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 r_len){
    // To be filled !
    return BNO055_SUCCESS;
}

s8 bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 wr_len){
    // To be filled !
    return BNO055_SUCCESS;
}

void delay_func(int delay_in_msec){
    k_msleep(delay_in_msec);
    return;
}



