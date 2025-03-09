/** ================================================================
 * @file    application/src/init/init.cpp
 *
 * @brief   Init.c define the function that check if the peripherals
 *          are ready to be used, or not.
 *
 * @date    19-02-2025
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

// Zephyr
#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>

// Custom libs
#include "init.h"
#include "../config.h"

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Initializer, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * DEFINING NODE ALIASES
 * -----------------------------------------------------------------
 */

#define PWM_SERVO_LEN DT_PROP_LEN(DT_PATH(wings, wings1), pwms)

// Serial related peripherals
#define UART_IMU DT_NODELABEL(uart1)
#define UART_GPS DT_NODELABEL(uart2)

// SPI related peripherals
#define // Configure SPI
#define EEPROM_0

// Count the number of cs used, and thus, the number of devices.
#define EEPROM_NB DT_PROP_LEN(DT_PATH(soc, peripheral_40000000, spi_a000), cs_gpios)

// I2C related peripherals
#define BAROMETER_0 DT_NODELABEL(barometer0)
#define ACCELEROMETER_0 DT_NODELABEL(accelerometer0)
#define ACCELEROMETER_1 DT_NODELABEL(accelerometer1)
#define EXPANDER_0 DT_NODELABEL(expander0)

/* -----------------------------------------------------------------
 * PRIVATES FUNCTIONS (For a single peripheral instance)
 * -----------------------------------------------------------------
 */

int CheckAGPIO(const struct gpio_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!gpio_is_ready_dt(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s:%d is not ready for now. Retrying in 5 ms...", Target->port->name, Target->pin);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s:%d is not working properly.", Target->port->name, Target->pin);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s:%d is working correctly !", Target->port->name, Target->pin);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAPWM(const struct pwm_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!pwm_is_ready_dt(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s:%d is not ready for now. Retrying in 5 ms...", Target->dev->name, Target->channel);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s:%d is not working properly.", Target->dev->name, Target->channel);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s:%d is working correctly !", Target->dev->name, Target->channel);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAnUART(const struct device *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!device_is_ready(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s is not ready for now. Retrying in 5 ms...", Target->name);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s is not working properly.", Target->name);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s is working correctly !", Target->name);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAnI2C(const struct i2c_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!device_is_ready(Target->bus))
        {
            if (k < 2)
            {
                LOG_WRN("%s@0x%xRW is not ready for now. Retrying in 5 ms...", Target->bus->name, Target->addr);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s@0x%xRW is not working properly.", Target->bus->name, Target->addr);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s@0x%xRW is working correctly !", Target->bus->name, Target->addr);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

int CheckAnSPI(const struct spi_dt_spec *Target)
{
    for (uint8_t k = 0; k < INIT_MAX_TRY; k++)
    {
        if (!spi_is_ready_dt(Target))
        {
            if (k < 2)
            {
                LOG_WRN("%s@%d is not ready for now. Retrying in 5 ms...",
                        Target->bus->name,
                        Target->config.cs.gpio.pin);
                k_msleep(5); // Small delay, if the kernel was too busy to initialize it...
            }
            else
            {
                LOG_ERR("%s@%d is not working properly.",
                        Target->bus->name,
                        Target->config.cs.gpio.pin);
                return 1;
            }
        }
        else
        {
            LOG_INF("%s@%d is working correctly !",
                    Target->bus->name,
                    Target->config.cs.gpio.pin);
            return 0;
        }
    }

    // Shall not get here...
    return 0xdeadbeef;
}

/* -----------------------------------------------------------------
 *  PRIVATE VARIABLES
 * -----------------------------------------------------------------
 */
// GPIOS
static bool _IsPeripheralResetInitialized = false;
static bool _IsModeInitialized = false;
static bool _IsLatchInitialized = false;
static bool _IsEnginesInitialized = false;
static bool _IsImuBootStatusInitialized = false;
static bool _IsImuActStatusInitialized = false;
static bool _IsImuIntInitialized = false;
static bool _IsGpsIntInitialized = false;
static bool _IsAccelIntInitialized = false;
static bool _IsInputsInitialized = false;

// PWMS
static bool _IsServoInitialized = false;
static bool _IsRGBInitialized = false;
static bool _IsParachuteInitialized = false;
static bool _IsBuzzerInitialized = false;

// I2C
static bool _IsBarometerInitialized = false;
static bool _IsExpanderInitialized = false;
static bool _IsAccelerometersInitialized = false;

// SPI
static bool _IsEepromInitialzed = false;

// UARTS
static bool _IsGPSInitialied = false;
static bool _IsIMUInitialized = false;

/* -----------------------------------------------------------------
 * FUNCTIONS TO CHECK IF THE PERIPHERAL IS OK
 * -----------------------------------------------------------------
 */

gpio_dt_spec *INIT_GetAGPIO(GPIOS Pin)
{
    // Switch over each GPIOS
    // We can't generlize due to the usage of macros by Zephyr kernel
    switch (Pin)
    {
    case GPIOS::PERIPHERAL_RESET:

        // First, check that the device was effectively free
        if (!_IsPeripheralResetInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *p_rst = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (p_rst == nullptr)
            {
                LOG_ERR("Failed to allocate memory for peripheral reset GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *p_rst = GPIO_DT_SPEC_GET(DT_ALIAS(peripheralreset), gpios);

            // Check that the device is working
            int err = CheckAGPIO(p_rst);
            if (!err)
            {
                LOG_ERR("Peripheral reset were fetched but failed kernel checks...");
                k_free(p_rst);
                return nullptr;
            }
            _IsPeripheralResetInitialized = true;
            return p_rst;
        }
        else
        {
            LOG_ERR("Peripheral Reset was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::MODE:

        // First, check that the device was effectively free
        if (!_IsModeInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *mode = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (mode == nullptr)
            {
                LOG_ERR("Failed to allocate memory for rocket mode GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *mode = GPIO_DT_SPEC_GET(DT_ALIAS(mode), gpios);

            // Check that the device is working
            int err = CheckAGPIO(mode);
            if (!err)
            {
                LOG_ERR("Rocket Mode were fetched but failed kernel checks...");
                k_free(mode);
                return nullptr;
            }
            _IsModeInitialized = true;
            return mode;
        }
        else
        {
            LOG_ERR("Rocket Mode was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::LATCH:

        // First, check that the device was effectively free
        if (!_IsLatchInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *p_latch = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (p_latch == nullptr)
            {
                LOG_ERR("Failed to allocate memory for rocket latch GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *p_latch = GPIO_DT_SPEC_GET(DT_ALIAS(latch), gpios);

            // Check that the device is working
            int err = CheckAGPIO(p_latch);
            if (!err)
            {
                LOG_ERR("Rocket Latch were fetched but failed kernel checks...");
                k_free(p_latch);
                return nullptr;
            }
            _IsLatchInitialized = true;
            return p_latch;
        }
        else
        {
            LOG_ERR("Rocket Latch was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::ENGINES:

#if (PWM_SERVO_LEN != 4)
#error "There is not 4 servo engines. Their init may fail !"
#endif

        // First, check that the device was effectively free
        if (!_IsEnginesInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *engine = (gpio_dt_spec *)k_malloc(PWM_SERVO_LEN * sizeof(gpio_dt_spec));
            if (engine == nullptr)
            {
                LOG_ERR("Failed to allocate memory for engines control GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            engine[0] = GPIO_DT_SPEC_GET(DT_ALIAS(engine1), gpios);
            engine[1] = GPIO_DT_SPEC_GET(DT_ALIAS(engine2), gpios);
            engine[2] = GPIO_DT_SPEC_GET(DT_ALIAS(engine3), gpios);
            engine[3] = GPIO_DT_SPEC_GET(DT_ALIAS(engine4), gpios);

            // Check that the device is working
            int err = 0;
            for (uint8_t k = 0; k < PWM_SERVO_LEN; k++)
                err -= CheckAGPIO(&engine[k]);

            if (!err)
            {
                LOG_ERR("Engines were fetched but failed kernel checks...");
                k_free(engine);
                return nullptr;
            }
            // Here, we return the tab directly, since it's already a pointer !
            _IsEnginesInitialized = true;
            return engine;
        }
        else
        {
            LOG_ERR("Engines was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::IMU_BOOTSTATUS:

        // First, check that the device was effectively free
        if (!_IsImuBootStatusInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *btstat = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (btstat == nullptr)
            {
                LOG_ERR("Failed to allocate memory for IMU boot GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *btstat = GPIO_DT_SPEC_GET(DT_ALIAS(imuboot), gpios);

            // Check that the device is working
            int err = CheckAGPIO(btstat);
            if (!err)
            {
                LOG_ERR("IMU Boot was fetched but failed kernel checks...");
                k_free(btstat);
                return nullptr;
            }
            _IsImuBootStatusInitialized = true;
            return btstat;
        }
        else
        {
            LOG_ERR("IMU Boot Reset was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::IMU_ACTSTATUS:

        // First, check that the device was effectively free
        if (!_IsImuActStatusInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *actstat = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (actstat == nullptr)
            {
                LOG_ERR("Failed to allocate memory for IMU status GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *actstat = GPIO_DT_SPEC_GET(DT_ALIAS(imustatus), gpios);

            // Check that the device is working
            int err = CheckAGPIO(actstat);
            if (!err)
            {
                LOG_ERR("IMU status was fetched but failed kernel checks...");
                k_free(actstat);
                return nullptr;
            }
            _IsImuActStatusInitialized = true;
            return actstat;
        }
        else
        {
            LOG_ERR("IMU status was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::IMU_INT:

        // First, check that the device was effectively free
        if (!_IsImuIntInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *imunit = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (imunit == nullptr)
            {
                LOG_ERR("Failed to allocate memory for IMU int GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *imunit = GPIO_DT_SPEC_GET(DT_ALIAS(intimu), gpios);

            // Check that the device is working
            int err = CheckAGPIO(imunit);
            if (!err)
            {
                LOG_ERR("IMU INT was fetched but failed kernel checks...");
                k_free(imunit);
                return nullptr;
            }
            _IsImuIntInitialized = true;
            return imunit;
        }
        else
        {
            LOG_ERR("IMU INT was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::GPS_INT:

        // First, check that the device was effectively free
        if (!_IsGpsIntInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *gpsint = (gpio_dt_spec *)k_malloc(sizeof(gpio_dt_spec));
            if (gpsint == nullptr)
            {
                LOG_ERR("Failed to allocate memory for IMU int GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *gpsint = GPIO_DT_SPEC_GET(DT_ALIAS(intgps), gpios);

            // Check that the device is working
            int err = CheckAGPIO(gpsint);
            if (!err)
            {
                LOG_ERR("GPS INT was fetched but failed kernel checks...");
                k_free(gpsint);
                return nullptr;
            }
            _IsGpsIntInitialized = true;
            return gpsint;
        }
        else
        {
            LOG_ERR("GPS INT was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::ACCEL_INT:

#if (ACCEL_NB != 2)
#error "Accelerometer number shall be 2 to ensure a correct initialization !"
#endif

        // First, check that the device was effectively free
        if (!_IsAccelIntInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *accel = (gpio_dt_spec *)k_malloc(ACCEL_NB * sizeof(gpio_dt_spec));
            if (accel == nullptr)
            {
                LOG_ERR("Failed to allocate memory for Accels int GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            accel[0] = GPIO_DT_SPEC_GET(DT_ALIAS(intaccel1), gpios);
            accel[1] = GPIO_DT_SPEC_GET(DT_ALIAS(intaccel2), gpios);

            // Check that the device is working
            int err = 0;
            for (uint8_t k = 0; k < ACCEL_NB; k++)
                err -= CheckAGPIO(&accel[k]);

            if (!err)
            {
                LOG_ERR("Accelerometer INTs was fetched but failed kernel checks...");
                k_free(accel);
                return nullptr;
            }
            // Tab is returned directly !
            _IsAccelIntInitialized = true;
            return accel;
        }
        else
        {
            LOG_ERR("Accelerometer INTs was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case GPIOS::INPUTS:

#if (INPUTS_NB != 3)
#error "Input number shall be equal to three to ensure a correct initialization of the gpios !"
#endif

        // First, check that the device was effectively free
        if (!_IsInputsInitialized)
        {
            // Allocate memory, and check
            gpio_dt_spec *inp = (gpio_dt_spec *)k_malloc(INPUTS_NB * sizeof(gpio_dt_spec));
            if (inp == nullptr)
            {
                LOG_ERR("Failed to allocate memory for Inputs GPIO structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            inp[0] = GPIO_DT_SPEC_GET(DT_ALIAS(int1), gpios);
            inp[1] = GPIO_DT_SPEC_GET(DT_ALIAS(int2), gpios);
            inp[2] = GPIO_DT_SPEC_GET(DT_ALIAS(int3), gpios);

            // Check that the device is working
            int err = 0;
            for (uint8_t k = 0; k < INPUTS_NB; k++)
                err -= CheckAGPIO(&inp[k]);

            if (!err)
            {
                LOG_ERR("Peripheral reset was fetched but failed kernel checks...");
                k_free(inp);
                return nullptr;
            }
            // Return the list directly
            _IsInputsInitialized = true;
            return inp;
        }
        else
        {
            LOG_ERR("Peripheral Reset was already initialized... Ignoring this request");
            return nullptr;
        }
        break;
    }

    // Can't get this, but who knows...
    return nullptr;
}
void INIT_FreeAGPIO(GPIOS Pin, gpio_dt_spec *GPIO)
{
    // First, set as High Z
    gpio_pin_configure_dt(GPIO, GPIO_DISCONNECTED);

    // Set the right value to false
    switch (Pin)
    {
    case GPIOS::PERIPHERAL_RESET:
        _IsPeripheralResetInitialized = false;
        break;
    case GPIOS::MODE:
        _IsModeInitialized = false;
        break;
    case GPIOS::LATCH:
        _IsLatchInitialized = false;
        break;
    case GPIOS::ENGINES:
        _IsEnginesInitialized = false;
        break;
    case GPIOS::IMU_BOOTSTATUS:
        _IsImuBootStatusInitialized = false;
        break;
    case GPIOS::IMU_ACTSTATUS:
        _IsImuActStatusInitialized = false;
        break;
    case GPIOS::IMU_INT:
        _IsImuIntInitialized = false;
        break;
    case GPIOS::GPS_INT:
        _IsGpsIntInitialized = false;
        break;
    case GPIOS::INPUTS:
        _IsInputsInitialized = false;
        break;
    case GPIOS::ACCEL_INT:
        _IsAccelIntInitialized = false;
        break;
    }

    // Free memory
    k_free(GPIO);
    return;
}

pwm_dt_spec *INIT_GetAPWM(PWMS Dev)
{
    switch (Dev)
    {

    case PWMS::SERVOS:
#if (PWM_SERVO_LEN != 4)
#error "Not enough Servos engines PWM channels were found on the device tree !"
#endif
        // First, check that the device was effectively free
        if (!_IsServoInitialized)
        {
            // Allocate memory, and check
            pwm_dt_spec *pwm = (pwm_dt_spec *)k_malloc(PWM_SERVO_LEN * sizeof(pwm_dt_spec));
            if (pwm == nullptr)
            {
                LOG_ERR("Failed to allocate enough memory for the PWM Servo structure !");
                return nullptr;
            }

            // Fill this memory with the device infos
            pwm[0] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(wings, wings1), 0);
            pwm[1] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(wings, wings1), 1);
            pwm[2] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(wings, wings1), 2);
            pwm[3] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(wings, wings1), 3);

            // Check that the device is working
            int err = 0;
            for (uint8_t k = 0; k < PWM_SERVO_LEN; k++)
                err -= CheckAPWM(&pwm[k]);

            if (!err)
            {
                LOG_ERR("Servo (PWM) peripheral was fetched but failed kernel checks...");
                k_free(pwm);
                return nullptr;
            }
            // Return the list directly
            _IsServoInitialized = true;
            return pwm;
        }
        else
        {
            LOG_ERR("Servo (PWM) peripheral was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case PWMS::RGB:
#if (DT_PROP_LEN(DT_PATH(rgb, rgb1), pwms) != 3)
#error "Not enough RGB PWM channels were found on the device tree !"
#endif
        // First, check that the device was effectively free
        if (!_IsRGBInitialized)
        {
            // Allocate memory, and check
            pwm_dt_spec *rgb = (pwm_dt_spec *)k_malloc(DT_PROP_LEN(DT_PATH(rgb, rgb1), pwms) * sizeof(pwm_dt_spec));
            if (rgb == nullptr)
            {
                LOG_ERR("Failed to allocate enough memory for the PWM RGB structure !");
                return nullptr;
            }

            // Fill this memory with the device infos
            rgb[0] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(rgb, rgb1), 0);
            rgb[1] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(rgb, rgb1), 1);
            rgb[2] = PWM_DT_SPEC_GET_BY_IDX(DT_PATH(rgb, rgb1), 2);

            // Check that the device is working
            int err = 0;
            for (uint8_t k = 0; k < PWM_SERVO_LEN; k++)
                err -= CheckAPWM(&rgb[k]);

            if (!err)
            {
                LOG_ERR("RGB (PWM) peripheral was fetched but failed kernel checks...");
                k_free(rgb);
                return nullptr;
            }
            // Return the list directly
            _IsRGBInitialized = true;
            return rgb;
        }
        else
        {
            LOG_ERR("RGB (PWM) peripheral was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case PWMS::PARACHUTE:
        // First, check that the device was effectively free
        if (!_IsParachuteInitialized)
        {
            // Allocate memory, and check
            pwm_dt_spec *para = (pwm_dt_spec *)k_malloc(sizeof(pwm_dt_spec));
            if (para == nullptr)
            {
                LOG_ERR("Failed to allocate enough memory for the PWM RGB structure !");
                return nullptr;
            }

            // Fill this memory with the device infos
            *para = PWM_DT_SPEC_GET(DT_NODELABEL(parachute));

            // Check that the device is working
            int err = CheckAPWM(para);

            if (!err)
            {
                LOG_ERR("Parachute (PWM) peripheral was fetched but failed kernel checks...");
                k_free(para);
                return nullptr;
            }
            // Return the list directly
            _IsParachuteInitialized = true;
            return para;
        }
        else
        {
            LOG_ERR("Parachute (PWM) peripheral was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case PWMS::BUZZER:
        // First, check that the device was effectively free
        if (!_IsBuzzerInitialized)
        {
            // Allocate memory, and check
            pwm_dt_spec *buzz = (pwm_dt_spec *)k_malloc(sizeof(pwm_dt_spec));
            if (buzz == nullptr)
            {
                LOG_ERR("Failed to allocate enough memory for the PWM RGB structure !");
                return nullptr;
            }

            // Fill this memory with the device infos
            *buzz = PWM_DT_SPEC_GET(DT_NODELABEL(buzzer));

            // Check that the device is working
            int err = CheckAPWM(buzz);

            if (!err)
            {
                LOG_ERR("Buzzer (PWM) peripheral was fetched but failed kernel checks...");
                k_free(buzz);
                return nullptr;
            }
            // Return the list directly
            _IsBuzzerInitialized = true;
            return buzz;
        }
        else
        {
            LOG_ERR("Buzzer (PWM) peripheral was already initialized... Ignoring this request");
            return nullptr;
        }
        break;
    }

    return nullptr;
}

void INIT_FreeAPWM(PWMS Dev, pwm_dt_spec *PWM)
{
    // First, set as inactive !
    pwm_set(PWM->dev,
            PWM->channel,
            PWM->period,
            0, // 0 ns pulse -> Off
            PWM->flags);

    // Set the right value to false
    switch (Dev)
    {
    case PWMS::SERVOS:
        _IsServoInitialized = false;
        break;
    case PWMS::RGB:
        _IsRGBInitialized = false;
        break;
    case PWMS::BUZZER:
        _IsBuzzerInitialized = false;
        break;
    case PWMS::PARACHUTE:
        _IsParachuteInitialized = false;
        break;
    }

    // Free memory
    k_free(PWM);
    return;
}

spi_dt_spec *INIT_GetAnSPI(SPIS Dev)
{
    switch (Dev)
    {
    case SPIS::EEPROM:
        // First, check that the device was effectively free
        if (!_IsEepromInitialzed)
        {
            // Allocate memory, and check
            spi_dt_spec *eep = (spi_dt_spec *)k_malloc(sizeof(spi_dt_spec));
            if (eep == nullptr)
            {
                LOG_ERR("Failed to allocate enough memory for the SPI EEPROM structure !");
                return nullptr;
            }

            // Fill this memory with the device infos
            *eep = SPI_DT_SPEC_GET(DT_NODELABEL(eeprom0),
                                   SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
                                   0);

            // Check that the device is working
            int err = CheckAnSPI(eep);

            if (!err)
            {
                LOG_ERR("EEPROM (SPI) peripheral was fetched but failed kernel checks...");
                k_free(eep);
                return nullptr;
            }
            // Return the list directly
            _IsEepromInitialzed = true;
            return eep;
        }
        else
        {
            LOG_ERR("EEPROM (SPI) peripheral was already initialized... Ignoring this request");
            return nullptr;
        }
        break;
    }
}

void INIT_FreeAnSPI(SPIS Dev, spi_dt_spec *SPI)
{
    // no shutdown procedure here...
    switch (Dev)
    {
    case SPIS::EEPROM:
        _IsEepromInitialzed = false;
        break;
    }

    // Free mem
    k_free(SPI);
    return;
}

i2c_dt_spec *INIT_GetAnI2C(I2CS Dev)
{
}
void INIT_FreeAnI2C(I2CS Dev, i2c_dt_spec *GPIO)
{
}

device *INIT_GetAnUART(UARTS Dev)
{
}

void INIT_FreeAnUART(UARTS Dev, device *UART)
{
}