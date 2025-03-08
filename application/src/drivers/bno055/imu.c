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

/* -----------------------------------------------------------------
* FUNCTIONS TO COMMAND AN IMU SENSOR
* -----------------------------------------------------------------
*/

bus_read(dev_addr, reg_addr, reg_data, r_len){
    // To be filled !
    return 0;
}

bus_write(dev_addr, reg_addr, reg_data, wr_len){
    // To be filled !
    return 0;
}

delay_func(delay_in_msec){
    k_msleep(delay_in_msec);
    return 0;
}



