/** ================================================================
 * @file    application/src/init/init.h
 *
 * @brief   Init.h is charged to fetch the DT node label from the
 *          devicetree and expose them to the C application code.
 *          This file also do some verification about the fetched
 *          structure to ensure they're usable.
 *
 * @date    09-03-2025
 *
 * @version 2.0.0
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

// Number of try before declaring peripheral out.
#define INIT_MAX_TRY 3

/* -----------------------------------------------------------------
 * DEVICETREE PARAMETERS FETCHING FOR OTHER FILE USAGE
 * -----------------------------------------------------------------
 */
#define PWM_SERVO_LEN DT_PROP_LEN(DT_PATH(wings, wings1), pwms)
#define PWM_RGB_LEN DT_PROP_LEN(DT_PATH(rgb, rgb1), pwms)
#define EEPROM_NB 1
#define ACCEL_NB 2
#define INPUTS_NB 3

/* -----------------------------------------------------------------
 * PROJECT CUSTOM ATTRIBUTES
 * -----------------------------------------------------------------
 */

// Declare a variable as unused, but used anyway...
#define ISR_ONLY_VARIABLE __attribute__((unused))
#define ISR_CALLBACK __attribute__((unused))
#define DEV_STRUCT __attribute__((unused))

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

typedef enum
{
    TIMER0,
    TIMER1,
    SAADC_TIMER
} TIMERS;

/* -----------------------------------------------------------------
 * FUNCTIONS TO GET HANDLER OF PERIPHERALS
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
struct gpio_dt_spec *INIT_GetAGPIO(GPIOS Pin);

/**
 * @brief   This function de-allocate a GPIO, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the GPIO soft-locked !
 *
 * @param   GPIO    A meùber of the GPIO enum, to free it's associated soft-lock
 * @param   Pin     A pointer to a gpio_dt_spec struct to be freed
 *
 * @return  Nothing
 */
void INIT_FreeAGPIO(GPIOS Pin, struct gpio_dt_spec *GPIO);

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
struct pwm_dt_spec *INIT_GetAPWM(PWMS Dev);

/**
 * @brief   This function de-allocate a PWM, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the PWM soft-locked !
 *
 * @param   Dev     A pointer to a pwm_dt_spec struct to be freed
 * @param   PWM     A meber of the PWMS enum, to free it's associated soft-lock
 *
 * @return  Nothing
 */
void INIT_FreeAPWM(PWMS Dev, struct pwm_dt_spec *PWM);

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  spi_dt_spec     Pointer to a struct that design THIS gpio !
 */
struct spi_dt_spec *INIT_GetAnSPI(SPIS Dev);

/**
 * @brief   This function de-allocate a SPIS, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the SPI soft-locked !
 *
 * @param   Dev     A pointer to a spi_dt_spec struct to be freed
 * @param   SPI     A meber of the GPIO enum, to free it's associated soft-lock
 *
 * @return  Nothing
 */
void INIT_FreeAnSPI(SPIS Dev, struct spi_dt_spec *SPI);

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  i2c_dt_spec     Pointer to a struct that design THIS gpio !
 */
struct i2c_dt_spec *INIT_GetAnI2C(I2CS Dev);

/**
 * @brief   This function de-allocate a I2C device, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the I2C device soft-locked !
 *
 * @param   Dev     A pointer to a i2c_dt_spec struct to be freed
 * @param   I2C     A member of the I2CS enum, to free it's associated soft-lock
 *
 * @return  Nothing
 */
void INIT_FreeAnI2C(I2CS Dev, struct i2c_dt_spec *I2C);

/**
 * @brief   This function initialize a GPIO, test it and return it for the user.
 *
 * @warning This function include a software lock for the GPIO, which mean that it CAN'T be initialized twice !
 *          This function is thus thread-proof !
 *
 * @param   Pin     A member of the GPIO enum, to design the wanted GPIO.
 *
 * @return  device  Pointer to a struct that design THIS gpio !
 */
const struct device *INIT_GetAnUART(UARTS Dev);

/**
 * @brief   This function de-allocate a UART, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the GPIO soft-locked !
 *
 * @param   Dev     A pointer to a device struct to be freed
 * @param   UART    A member of the UART enum, to free it's associated soft-lock
 *
 * @return  Nothing
 */
void INIT_FreeAnUART(UARTS Dev, struct device *UART);
/**
 * @brief   This function check the USB peripheral and initialize it.
 *          The USB port is then initialized as a virtual com port, and
 *          is used as a logger output.
 *
 * @return  0   Suceed
 * @return -1   Fail
 */
int INIT_CheckUSB();

/**
 * @brief   This function return an instance of a timer in the nRFX driver mode.
 *
 * @param   Dev A member of the TIMERS enum to describe the right timer
 *
 * @return  nrfx_timer_t    Pointer to a struct that design THIS gpio !
 */
nrfx_timer_t *INIT_GetATimer(TIMERS Dev);

/**
 * @brief   This function de-allocate a Timer, making it available for a further usage !
 *
 * @warning The soft lock is only cleared by it's name, passing an incorrect reference may free
 *          another memory struct while maintaining the GPIO soft-locked !
 *
 * @param   Dev     A pointer to a nrfx_timer_t struct to be freed
 * @param   UART    A member of the TIMERS enum, to free it's associated soft-lock
 *
 * @return  Nothing
 */
void INIT_FreeATimer(TIMERS Dev, nrfx_timer_t *Timer);

#endif /* DEF_INIT*/
