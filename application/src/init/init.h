/** ================================================================
 * @file    application/src/init/init.h
 *
 * @brief   Init.h is charged to fetch the DT node label from the
 *          devicetree and expose them to the C application code.
 *          This file also do some verification about the fetched 
 *          structure to ensure they're usable. 
 *
 * @date    19-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

// Include once
#ifndef DEF_INIT
    #define DEF_INIT

    /* -----------------------------------------------------------------
    * INCLUDING LIBS
    * -----------------------------------------------------------------
    */

    // Zephyr driver libs
    #include <zephyr/drivers/gpio.h>
    #include <zephyr/drivers/pwm.h>
    #include <zephyr/devicetree.h>
    #include <zephyr/drivers/spi.h>
    #include <zephyr/drivers/i2c.h>
    #include <zephyr/usb/usb_device.h>
    #include <zephyr/usb/usbd.h>
    #include <zephyr/kernel.h>
    #include <zephyr/device.h>

    // nRFX
    #include <nrfx_timer.h>

    /* -----------------------------------------------------------------
    * DEFINING NODE ALIASES
    * -----------------------------------------------------------------
    */

    // GPIOS
    // Outputs
    #define PERIPH_RESET                DT_ALIAS(peripheralreset)
    #define ROCKET_LATCH                DT_ALIAS(latch)
    #define ENGINE_1                    DT_ALIAS(engine1)
    #define ENGINE_2                    DT_ALIAS(engine2)
    #define ENGINE_3                    DT_ALIAS(engine3)
    #define ENGINE_4                    DT_ALIAS(engine4)
    // Inputs
    #define IMU_BOOT                    DT_ALIAS(imuboot)
    #define IMU_STATUS                  DT_ALIAS(imustatus)
    #define ROCKET_MODE                 DT_ALIAS(mode)
    // Interruptibles inputs
    #define INT_IMU                     DT_ALIAS(intimu)
    #define INT_GPS                     DT_ALIAS(intgps)
    #define INT_ACCEL1                  DT_ALIAS(intaccel1)
    #define INT_ACCEL2                  DT_ALIAS(intaccel2)
    #define INT_1                       DT_ALIAS(int1)
    #define INT_2                       DT_ALIAS(int2)
    #define INT_3                       DT_ALIAS(int3)

    // PWM related peripherals
    #define WINGS_1                     DT_PATH(wings, wings1)
    #define RGB_1                       DT_PATH(rgb, rgb1)
    #define BUZZER                      DT_NODELABEL(buzzer)
    #define PARACHUTE                   DT_NODELABEL(parachute)

    // Fetching lengh of different DT PROP
    #define PWM_RGB_LEN                 DT_PROP_LEN(RGB_1, pwms)
    #define PWM_SERVO_LEN               DT_PROP_LEN(WINGS_1, pwms)

    // Serial related peripherals
    #define UART_IMU                    DT_NODELABEL(uart1)
    #define UART_GPS                    DT_NODELABEL(uart2)

    // SPI related peripherals
    #define SPIOP	                    SPI_WORD_SET(8) | SPI_TRANSFER_MSB // Configure SPI
    #define EEPROM_0                    DT_NODELABEL(eeprom0)

    // Count the number of cs used, and thus, the number of devices.
    #define EEPROM_NB                   DT_PROP_LEN(DT_PATH(soc, peripheral_40000000, spi_a000), cs_gpios)

    // I2C related peripherals
    #define BAROMETER_0                 DT_NODELABEL(barometer0)
    #define ACCELEROMETER_0             DT_NODELABEL(accelerometer0)
    #define ACCELEROMETER_1             DT_NODELABEL(accelerometer1)
    #define EXPANDER_0                  DT_NODELABEL(expander0)

    #define ACCEL_NB                    2

    // Module settings
    #define INIT_MAX_TRY                3 // Number of try before declaring peripheral out.

    // Timers
    #define SAADC_TIMER_NUMBER          2 // Timer2 will be used for the SAADC sampling triggering.

    /* -----------------------------------------------------------------
    * FETCHING C STRUCTS THAT DESCRIBE EACH PERIPHERALS
    * -----------------------------------------------------------------
    */

    // GPIOS
    // Outputs
    static const struct gpio_dt_spec    peripheral_reset            =       GPIO_DT_SPEC_GET(PERIPH_RESET, gpios);
    static const struct gpio_dt_spec    rocket_latch                =       GPIO_DT_SPEC_GET(ROCKET_LATCH, gpios);
    static const struct gpio_dt_spec    engines[4]                  =  {    GPIO_DT_SPEC_GET(ENGINE_1, gpios),
                                                                            GPIO_DT_SPEC_GET(ENGINE_2, gpios),
                                                                            GPIO_DT_SPEC_GET(ENGINE_3, gpios),
                                                                            GPIO_DT_SPEC_GET(ENGINE_4, gpios)};
    // Inputs
    static const struct gpio_dt_spec    imu_boot                    =       GPIO_DT_SPEC_GET(IMU_BOOT, gpios);  
    static const struct gpio_dt_spec    imu_status                  =       GPIO_DT_SPEC_GET(IMU_STATUS, gpios);  
    static const struct gpio_dt_spec    rocket_mode                 =       GPIO_DT_SPEC_GET(ROCKET_MODE, gpios);  
    // Interruptibles inputs
    static const struct gpio_dt_spec    imu_int                     =       GPIO_DT_SPEC_GET(INT_IMU, gpios); 
    static const struct gpio_dt_spec    gps_int                     =       GPIO_DT_SPEC_GET(INT_GPS, gpios); 
    static const struct gpio_dt_spec    accel1_int                  =       GPIO_DT_SPEC_GET(INT_ACCEL1, gpios); 
    static const struct gpio_dt_spec    accel2_int                  =       GPIO_DT_SPEC_GET(INT_ACCEL2, gpios);  
    static const struct gpio_dt_spec    inputs[3]                   = {     GPIO_DT_SPEC_GET(INT_1, gpios),
                                                                            GPIO_DT_SPEC_GET(INT_2, gpios),
                                                                            GPIO_DT_SPEC_GET(INT_3, gpios)};    


    // UARTS
    static const struct device          *uart_imu                   =       DEVICE_DT_GET(UART_IMU);
    static const struct device          *uart_gps                   =       DEVICE_DT_GET(UART_GPS);

    // Servo engines (wings) commands
    static const struct pwm_dt_spec     pwm_wings[PWM_SERVO_LEN]    = {     PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 0),
                                                                            PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 1),
                                                                            PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 2),
                                                                            PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 3)};

    static const struct pwm_dt_spec     pwm_rgb[PWM_RGB_LEN]        = {     PWM_DT_SPEC_GET_BY_IDX(RGB_1, 0), 
                                                                            PWM_DT_SPEC_GET_BY_IDX(RGB_1, 1), 
                                                                            PWM_DT_SPEC_GET_BY_IDX(RGB_1, 2)};

    static const struct pwm_dt_spec     pwm_buzzer                  =       PWM_DT_SPEC_GET(BUZZER);
    static const struct pwm_dt_spec     pwm_parachute               =       PWM_DT_SPEC_GET(PARACHUTE);

    // SPI devices
    static const struct spi_dt_spec     spi_eeproms                 =       SPI_DT_SPEC_GET(EEPROM_0, SPIOP, 0);

    // I2C devices
    static const struct i2c_dt_spec     i2c_barometer               =       I2C_DT_SPEC_GET(BAROMETER_0);
    static const struct i2c_dt_spec     i2c_expander                =       I2C_DT_SPEC_GET(EXPANDER_0);
    static const struct i2c_dt_spec     i2c_accels[ACCEL_NB]        = {     I2C_DT_SPEC_GET(ACCELEROMETER_0),
                                                                            I2C_DT_SPEC_GET(ACCELEROMETER_1)};

    // Timers
    static              nrfx_timer_t    saadc_timer                 =       NRFX_TIMER_INSTANCE(SAADC_TIMER_NUMBER);

    /* -----------------------------------------------------------------
    * FUNCTIONS TO CHECK IF THE PERIPHERAL IS OK
    * -----------------------------------------------------------------
    */

    /**
     * @brief   This function check if all of the leds are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0   All peripherals are working
     * @return -1   A peripheral is not working
     * @return ...
     * @return -n   n peipherals are not working
     */
    int INIT_CheckGPIO();

    /**
     * @brief   This function check if all of the PWMs are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0   All peripherals are working
     * @return -1   A peripheral is not working
     * @return ...
     * @return -n   n peipherals are not working
     */
    int INIT_CheckPWM();
    
    /**
     * @brief   This function check if all of the UARTs are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0   All peripherals are working
     * @return -1   A peripheral is not working
     * @return ...
     * @return -n   n peipherals are not working
     */
    int INIT_CheckUART();

    /**
     * @brief   This function check if all of the I2C are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0   All peripherals are working
     * @return -1   A peripheral is not working
     * @return ...
     * @return -n   n peipherals are not working
     */
    int INIT_CheckI2C();
    
    /**
     * @brief   This function check if all of the SPI are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0   All peripherals are working
     * @return -1   A peripheral is not working
     * @return ...
     * @return -n   n peipherals are not working
     */
    int INIT_CheckSPI();

    /**
     * @brief   This function check the USB peripheral and initialize it.
     *          The USB port is then initialized as a virtual com port, and
     *          is used as a logger output.
     * 
     * @return  0
     */
    int INIT_CheckUSB();

#endif

