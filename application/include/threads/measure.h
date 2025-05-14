/** ================================================================
 * @file    application/include/threads/measure.h
 *
 * @brief   This file declare the measurement engine of the rocket
 *
 * @date    18-04-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

 #ifndef _DEF_THREADS_MEASURE
 #define _DEF_THREADS_MEASURE
 
 /* -----------------------------------------------------------------
  * LIBS
  * -----------------------------------------------------------------
  */
 #include <zephyr/kernel.h>
 
 /* -----------------------------------------------------------------
  * STRUCTS
  * -----------------------------------------------------------------
  */
 
 struct measure_p1
 {
    
    struct k_fifo &barom_data1;
    struct k_fifo &barom_data2;
    struct k_fifo &barom_data3;

    struct k_fifo &adc_data1;
    struct k_fifo &adc_data2;
    struct k_fifo &adc_data3;

    struct k_fifo &imu_data1;
    struct k_fifo &imu_data2;
    struct k_fifo &imu_data3;

    struct k_fifo &gps_data1;
    struct k_fifo &gps_data2;

    struct k_fifo &accels_data1;
    struct k_fifo &accels_data2;
 };
 
 /* -----------------------------------------------------------------
  * THREADS
  * -----------------------------------------------------------------
  */
 
 void thread_measure(void *p1, void *p2, void *p3);
 
 #endif /* _DEF_THREADS_MEASURE */