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
 * MODULE SETTINGS
 * -----------------------------------------------------------------
 */

#define ACCEL_NB 2
#define INPUTS_NB 3

// Module settings
#define INIT_MAX_TRY 3 // Number of try before declaring peripheral out.

// Timers
#define SAADC_TIMER_NUMBER 2 // Timer2 will be used for the SAADC sampling triggering.

/* -----------------------------------------------------------------
 * ENUMS
 * -----------------------------------------------------------------
 */

typedef enum
{
    PERIPHERAL_RESET,
    LATCH,
    ENGINES,
    IMU_BOOTSTATUS,
    IMU_ACTSTATUS,
    MODE,
    IMU_INT,
    GPS_INT,
    ACCEL_INT,
    INPUTS
} GPIOS;

typedef enum
{
    IMU,
    GPS
} UARTS;

typedef enum
{
    SERVOS,
    RGB,
    BUZZER,
    PARACHUTE
} PWMS;

typedef enum
{
    EEPROM // single element typedef to remains compatible with other peripherals
} SPIS;

typedef enum
{
    BAROMETER,
    ACCELEROMETERS,
    EXPANDER
} I2CS;

/* -----------------------------------------------------------------
 * FETCHING C STRUCTS THAT DESCRIBE EACH PERIPHERALS
 * -----------------------------------------------------------------
 */

// UARTS
static const struct device *uart_imu = DEVICE_DT_GET(UART_IMU);
static const struct device *uart_gps = DEVICE_DT_GET(UART_GPS);

// SPI devices
static const struct spi_dt_spec spi_eeproms =

    // I2C devices
    static const struct i2c_dt_spec i2c_barometer = I2C_DT_SPEC_GET(BAROMETER_0);
static const struct i2c_dt_spec i2c_expander = I2C_DT_SPEC_GET(EXPANDER_0);
static const struct i2c_dt_spec i2c_accels[ACCEL_NB] = {I2C_DT_SPEC_GET(ACCELEROMETER_0),
                                                        I2C_DT_SPEC_GET(ACCELEROMETER_1)};

// Timers
static nrfx_timer_t saadc_timer = NRFX_TIMER_INSTANCE(SAADC_TIMER_NUMBER);

/* -----------------------------------------------------------------
 * FUNCTIONS TO CHECK IF THE PERIPHERAL IS OK
 * -----------------------------------------------------------------
 */

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  gpio_dt_spec    Pointer to a struct that design THIS gpio !
 */
gpio_dt_spec *INIT_GetAGPIO(GPIOS Pin);

/**
 * @brief   This function de-allocate a GPIO, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the GPIO soft-locked !
 *
 * @param   GPIO    A meùber of the GPIO enum, to free it's associated soft-lock
 * @param   Pin     A pointer to a gpio_dt_spec struct to be freed
 */
void INIT_FreeAGPIO(GPIOS Pin, gpio_dt_spec *GPIO);

/**
 * @brief   This function initialize a PWM, test it and return it for the user.
 *
 * @warning This function include a software lock for the PWM, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the PWMS enum, to design the wanted PWM group.
 *
 * @return  pwm_dt_spec     Pointer to a struct that design THIS gpio !
 */
pwm_dt_spec *INIT_GetAPWM(PWMS Dev);

/**
 * @brief   This function de-allocate a PWM, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the PWM soft-locked !
 *
 * @param   Dev     A pointer to a pwm_dt_spec struct to be freed
 * @param   PWM     A meber of the PWMS enum, to free it's associated soft-lock
 */
void INIT_FreeAPWM(PWMS Dev, pwm_dt_spec *PWM);

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  gpio_dt_spec    Pointer to a struct that design THIS gpio !
 */
spi_dt_spec *INIT_GetAnSPI(SPIS Dev);

/**
 * @brief   This function de-allocate a SPIS, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the SPI soft-locked !
 *
 * @param   Dev     A pointer to a spi_dt_spec struct to be freed
 * @param   SPI     A meber of the GPIO enum, to free it's associated soft-lock
 */
void INIT_FreeAnSPI(SPIS Dev, spi_dt_spec *SPI);

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  gpio_dt_spec    Pointer to a struct that design THIS gpio !
 */
i2c_dt_spec *INIT_GetAnI2C(I2CS Dev);

/**
 * @brief   This function de-allocate a I2C device, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the I2C device soft-locked !
 *
 * @param   Dev     A pointer to a i2c_dt_spec struct to be freed
 * @param   I2C    A meber of the I2CS enum, to free it's associated soft-lock
 */
void INIT_FreeAnI2C(I2CS Dev, i2c_dt_spec *I2C);

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  gpio_dt_spec    Pointer to a struct that design THIS gpio !
 */
device *INIT_GetAnUART(UARTS Dev);

/**
 * @brief   This function de-allocate a UART, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the GPIO soft-locked !
 *
 * @param   Dev     A pointer to a device struct to be freed
 * @param   UART    A meber of the UART enum, to free it's associated soft-lock
 */
void INIT_FreeAnUART(UARTS Dev, device *UART);

#endif /* DEF_INIT*/
