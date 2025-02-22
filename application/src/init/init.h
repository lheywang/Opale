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

    /* -----------------------------------------------------------------
    * DEFINING NODE ALIASES
    * -----------------------------------------------------------------
    */

    // Onboard leds
    #define LED0                        DT_ALIAS(led0)

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
    #define EEPROM_1                    DT_NODELABEL(eeprom1)
    #define EEPROM_2                    DT_NODELABEL(eeprom2)

    #define EEPROM_NB                   3 // There is 3 eeprom on the board

    // I2C related peripherals
    #define BAROMETER_0                 DT_NODELABEL(barometer0)
    #define ACCELEROMETER_0             DT_NODELABEL(accelerometer0)
    #define ACCELEROMETER_1             DT_NODELABEL(accelerometer1)
    #define EXPANDER_0                  DT_NODELABEL(expander0)

    #define ACCEL_NB                    2 // There is two optionnal I2C accelerometers on the board.
    


    // Module settings
    #define INIT_MAX_TRY                3 // Number of try before declaring peripheral out.

    /* -----------------------------------------------------------------
    * FETCHING C STRUCTS THAT DESCRIBE EACH PERIPHERALS
    * -----------------------------------------------------------------
    */

    // Onboard leds
    static const struct gpio_dt_spec    led                      =      GPIO_DT_SPEC_GET(LED0, gpios);
    // UARTS
    static const struct device          *uart_imu                =      DEVICE_DT_GET(UART_IMU);
    static const struct device          *uart_gps                =      DEVICE_DT_GET(UART_GPS);

    // Servo engines (wings) commands
    static const struct pwm_dt_spec     pwm_wings[PWM_SERVO_LEN] = {    PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 0),
                                                                        PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 1),
                                                                        PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 2),
                                                                        PWM_DT_SPEC_GET_BY_IDX(WINGS_1, 3)};

    static const struct pwm_dt_spec     pwm_rgb[PWM_RGB_LEN]     = {    PWM_DT_SPEC_GET_BY_IDX(RGB_1, 0), 
                                                                        PWM_DT_SPEC_GET_BY_IDX(RGB_1, 1), 
                                                                        PWM_DT_SPEC_GET_BY_IDX(RGB_1, 2)};

    static const struct pwm_dt_spec     pwm_buzzer               =      PWM_DT_SPEC_GET(BUZZER);
    static const struct pwm_dt_spec     pwm_parachute            =      PWM_DT_SPEC_GET(PARACHUTE);

    // SPI devices
    // static const struct spi_dt_spec     spi_eeproms[EEPROM_NB]   = {    SPI_DT_SPEC_GET(EEPROM_0, SPIOP, 0),
    //                                                                     SPI_DT_SPEC_GET(EEPROM_1, SPIOP, 0),
    //                                                                     SPI_DT_SPEC_GET(EEPROM_2, SPIOP, 0)};

    // I2C devices
    static const struct i2c_dt_spec     i2c_barometer             =     I2C_DT_SPEC_GET(BAROMETER_0);
    static const struct i2c_dt_spec     i2c_expander              =     I2C_DT_SPEC_GET(EXPANDER_0);
    static const struct i2c_dt_spec     i2c_accels[ACCEL_NB]      = {   I2C_DT_SPEC_GET(ACCELEROMETER_0),
                                                                        I2C_DT_SPEC_GET(ACCELEROMETER_1)};
    // DT_PROP(BAROMETER0_NODE, status); --> To check while init

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
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckLedsPeripherals();

    /**
     * @brief   This function check if all of the PWMs are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckPWMPeripherals();
    
    /**
     * @brief   This function check if all of the UARTs are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckUARTPeripherals();

    /**
     * @brief   This function check if all of the I2C are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckI2CPeripherals();
    
    /**
     * @brief   This function check if all of the SPI are working properly
     * 
     * @warning Theses function only check the internal peripheral. 
     *          Not the physical actuator.
     * 
     * @return  0 : All peripherals are working
     * @return -1 : A peripheral is not working
     * @return ...
     * @return -n : n peipherals are not working
     */
    int CheckSPIPeripherals();

#endif

