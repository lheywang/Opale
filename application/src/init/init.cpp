/** ================================================================
 * @file    application/src/init/init.cpp
 *
 * @brief   Init.cpp define the function that check if the peripherals
 *          are ready to be used, or not.
 *
 * @date    09-03-2025
 *
 * @version 2.0.0
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
static bool _IsGPSInitialized = false;
static bool _IsIMUInitialized = false;

// TIMERS
static bool _IsTimer1Initialized = false;
static bool _IsTimer2Initialized = false;
static bool _IsTimer3Initialized = false;

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
            if (err != 0)
            {
                LOG_ERR("Peripheral reset were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)p_rst);
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
            if (err != 0)
            {
                LOG_ERR("Rocket Mode were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)mode);
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
            if (err != 0)
            {
                LOG_ERR("Rocket Latch were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)p_latch);
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
            {
                err -= CheckAGPIO(&engine[k]);
            }

            if (err != 0)
            {
                LOG_ERR("Engines were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)engine);
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
            if (err != 0)
            {
                LOG_ERR("IMU Boot was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)btstat);
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
            if (err != 0)
            {
                LOG_ERR("IMU status was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)actstat);
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
            if (err != 0)
            {
                LOG_ERR("IMU INT was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)imunit);
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
            if (err != 0)
            {
                LOG_ERR("GPS INT was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)gpsint);
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
            {
                err -= CheckAGPIO(&accel[k]);
            }

            if (err != 0)
            {
                LOG_ERR("Accelerometer INTs was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)accel);
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
            {
                err -= CheckAGPIO(&inp[k]);
            }

            if (err != 0)
            {
                LOG_ERR("Peripheral reset was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)inp);
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
        /*
         * Add here your de-init code !
         */
        _IsPeripheralResetInitialized = false;
        break;
    case GPIOS::MODE:
        /*
         * Add here your de-init code !
         */
        _IsModeInitialized = false;
        break;
    case GPIOS::LATCH:
        /*
         * Add here your de-init code !
         */
        _IsLatchInitialized = false;
        break;
    case GPIOS::ENGINES:
        /*
         * Add here your de-init code !
         */
        _IsEnginesInitialized = false;
        break;
    case GPIOS::IMU_BOOTSTATUS:
        /*
         * Add here your de-init code !
         */
        _IsImuBootStatusInitialized = false;
        break;
    case GPIOS::IMU_ACTSTATUS:
        /*
         * Add here your de-init code !
         */
        _IsImuActStatusInitialized = false;
        break;
    case GPIOS::IMU_INT:
        /*
         * Add here your de-init code !
         */
        _IsImuIntInitialized = false;
        break;
    case GPIOS::GPS_INT:
        /*
         * Add here your de-init code !
         */
        _IsGpsIntInitialized = false;
        break;
    case GPIOS::INPUTS:
        /*
         * Add here your de-init code !
         */
        _IsInputsInitialized = false;
        break;
    case GPIOS::ACCEL_INT:
        /*
         * Add here your de-init code !
         */
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
            {
                err -= CheckAPWM(&pwm[k]);
            }

            if (err != 0)
            {
                LOG_ERR("Servo (PWM) peripheral was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)pwm);
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
            for (uint8_t k = 0; k < DT_PROP_LEN(DT_PATH(rgb, rgb1), pwms); k++)
            {
                err -= CheckAPWM(&rgb[k]);
            }

            if (err != 0)
            {
                LOG_ERR("RGB (PWM) peripheral was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)rgb);
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

            if (err != 0)
            {
                LOG_ERR("Parachute (PWM) peripheral was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)para);
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

            if (err != 0)
            {
                LOG_ERR("Buzzer (PWM) peripheral was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)buzz);
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
        /*
         * Add here your de-init code !
         */
        _IsServoInitialized = false;
        break;
    case PWMS::RGB:
        /*
         * Add here your de-init code !
         */
        _IsRGBInitialized = false;
        break;
    case PWMS::BUZZER:
        /*
         * Add here your de-init code !
         */
        _IsBuzzerInitialized = false;
        break;
    case PWMS::PARACHUTE:
        /*
         * Add here your de-init code !
         */
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

            if (err != 0)
            {
                LOG_ERR("EEPROM (SPI) peripheral was fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)eep);
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

    // Shall never get here !
    return nullptr;
}

void INIT_FreeAnSPI(SPIS Dev, spi_dt_spec *SPI)
{
    // no shutdown procedure here...
    switch (Dev)
    {
    case SPIS::EEPROM:
        /*
         * Add here your de-init code !
         */
        _IsEepromInitialzed = false;
        break;
    }

    // Free mem
    k_free(SPI);
    return;
}

i2c_dt_spec *INIT_GetAnI2C(I2CS Dev)
{
    switch (Dev)
    {
    case I2CS::ACCELEROMETERS:

        // First, check that the device was effectively free
        if (!_IsAccelerometersInitialized)
        {
            // Allocate memory, and check
            i2c_dt_spec *accel = (i2c_dt_spec *)k_malloc(ACCEL_NB * sizeof(i2c_dt_spec));
            if (accel == nullptr)
            {
                LOG_ERR("Failed to allocate memory for the I2C accelerometers structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            accel[0] = I2C_DT_SPEC_GET(DT_NODELABEL(accelerometer0));
            accel[1] = I2C_DT_SPEC_GET(DT_NODELABEL(accelerometer1));

            // Check that the device is working
            int err = 0;
            for (uint8_t k = 0; k < ACCEL_NB; k++)
            {
                err -= CheckAnI2C(&accel[k]);
            }

            if (err != 0)
            {
                LOG_ERR("I2C accelerometers were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)accel);
                return nullptr;
            }
            _IsAccelerometersInitialized = true;
            return accel;
        }
        else
        {
            LOG_ERR("I2C accelerometers was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case I2CS::BAROMETER:

        // First, check that the device was effectively free
        if (!_IsBarometerInitialized)
        {
            // Allocate memory, and check
            i2c_dt_spec *barom = (i2c_dt_spec *)k_malloc(sizeof(i2c_dt_spec));
            if (barom == nullptr)
            {
                LOG_ERR("Failed to allocate memory for barometer I2C structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *barom = I2C_DT_SPEC_GET(DT_NODELABEL(barometer0));

            // Check that the device is working
            int err = CheckAnI2C(barom);
            if (err != 0)
            {
                LOG_ERR("Barometer (I2C) were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)barom);
                return nullptr;
            }
            _IsBarometerInitialized = true;
            return barom;
        }
        else
        {
            LOG_ERR("Barometer (I2C) was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case I2CS::EXPANDER:

        // First, check that the device was effectively free
        if (!_IsExpanderInitialized)
        {
            // Allocate memory, and check
            i2c_dt_spec *expa = (i2c_dt_spec *)k_malloc(sizeof(i2c_dt_spec));
            if (expa == nullptr)
            {
                LOG_ERR("Failed to allocate memory for barometer I2C structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *expa = I2C_DT_SPEC_GET(DT_NODELABEL(expander0));

            // Check that the device is working
            int err = CheckAnI2C(expa);
            if (err != 0)
            {
                LOG_ERR("Barometer (I2C) were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)expa);
                return nullptr;
            }
            _IsExpanderInitialized = true;
            return expa;
        }
        else
        {
            LOG_ERR("Barometer (I2C) was already initialized... Ignoring this request");
            return nullptr;
        }
        break;
    }

    // Shall not get here, but anyway...
    return nullptr;
}
void INIT_FreeAnI2C(I2CS Dev, i2c_dt_spec *I2C)
{
    // Update status of the I2C device
    switch (Dev)
    {
    case I2CS::ACCELEROMETERS:
        /*
         * Add here your de-init code !
         */
        _IsAccelerometersInitialized = false;
        break;

    case I2CS::BAROMETER:
        /*
         * Add here your de-init code !
         */
        _IsBarometerInitialized = false;
        break;

    case I2CS::EXPANDER:
        /*
         * Add here your de-init code !
         */
        _IsExpanderInitialized = false;
        break;
    }

    // Freed memory
    k_free(I2C);
    return;
}

const device *INIT_GetAnUART(UARTS Dev)
{
    switch (Dev)
    {
    case UARTS::GPS:

        // First, check that the device was effectively free
        if (!_IsGPSInitialized)
        {
            // Allocate memory, and check
            const device *p_gps = (device *)k_malloc(sizeof(device));
            if (p_gps == nullptr)
            {
                LOG_ERR("Failed to allocate memory for GPS UART structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            p_gps = DEVICE_DT_GET(DT_NODELABEL(uart2));

            // Check that the device is working
            int err = CheckAnUART(p_gps);
            if (err != 0)
            {
                LOG_ERR("GPS UART were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)p_gps);
                return nullptr;
            }
            _IsGPSInitialized = true;
            return p_gps;
        }
        else
        {
            LOG_ERR("GPS UART was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case UARTS::IMU:

        // First, check that the device was effectively free
        if (!_IsIMUInitialized)
        {
            // Allocate memory, and check
            const device *p_imu = (device *)k_malloc(sizeof(device));
            if (p_imu == nullptr)
            {
                LOG_ERR("Failed to allocate memory for IMU UART structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            p_imu = DEVICE_DT_GET(DT_NODELABEL(uart2));

            // Check that the device is working
            int err = CheckAnUART(p_imu);
            if (err != 0)
            {
                LOG_ERR("IMU UART were fetched but failed kernel checks... Freeing memory and aborting !");
                k_free((void *)p_imu);
                return nullptr;
            }
            _IsIMUInitialized = true;
            return p_imu;
        }
        else
        {
            LOG_ERR("IMU UART was already initialized... Ignoring this request");
            return nullptr;
        }
        break;
    }

    // Shall not get here, but anyway...
    return nullptr;
}

void INIT_FreeAnUART(UARTS Dev, device *UART)
{
    switch (Dev)
    {
    case UARTS::GPS:
        /*
         * Add here your de-init code !
         */
        _IsGpsIntInitialized = false;
        break;

    case UARTS::IMU:
        /*
         * Add here your de-init code !
         */
        _IsIMUInitialized = false;
        break;
    }

    // Free memory and exit
    k_free(UART);
    return;
}

int INIT_CheckUSB()
{
    if (usb_enable(NULL))
        return -1;
    return 0;
}

nrfx_timer_t *INIT_GetATimer(TIMERS Dev)
{
    switch (Dev)
    {
    case TIMERS::TIMER0:
        // First, check that the device was effectively free
        if (!_IsTimer1Initialized)
        {
            // Allocate memory, and check
            nrfx_timer_t *p_tmr1 = (nrfx_timer_t *)k_malloc(sizeof(nrfx_timer_t));
            if (p_tmr1 == nullptr)
            {
                LOG_ERR("Failed to allocate memory for Timer 0 structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *p_tmr1 = NRFX_TIMER_INSTANCE(0);

            // No checks can be done !

            _IsTimer1Initialized = true;
            return p_tmr1;
        }
        else
        {
            LOG_ERR("Timer 1 was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case TIMERS::TIMER1:
        // First, check that the device was effectively free
        if (!_IsTimer2Initialized)
        {
            // Allocate memory, and check
            nrfx_timer_t *p_tmr1 = (nrfx_timer_t *)k_malloc(sizeof(nrfx_timer_t));
            if (p_tmr1 == nullptr)
            {
                LOG_ERR("Failed to allocate memory for Timer 1 structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *p_tmr1 = NRFX_TIMER_INSTANCE(1);

            // No checks can be done !

            _IsTimer2Initialized = true;
            return p_tmr1;
        }
        else
        {
            LOG_ERR("Timer 0 was already initialized... Ignoring this request");
            return nullptr;
        }
        break;

    case TIMERS::SAADC_TIMER:
        // First, check that the device was effectively free
        if (!_IsTimer3Initialized)
        {
            // Allocate memory, and check
            nrfx_timer_t *p_tmr1 = (nrfx_timer_t *)k_malloc(sizeof(nrfx_timer_t));
            if (p_tmr1 == nullptr)
            {
                LOG_ERR("Failed to allocate memory for Timer 2 structure");
                return nullptr;
            }

            // Fill this memory with the device infos
            *p_tmr1 = NRFX_TIMER_INSTANCE(2);

            // No checks can be done !

            _IsTimer3Initialized = true;
            return p_tmr1;
        }
        else
        {
            LOG_ERR("Timer 2 was already initialized... Ignoring this request");
            return nullptr;
        }
        break;
    }

    // Shall not get here, but anyway...
    return nullptr;
}

void INIT_FreeATimer(TIMERS Dev, nrfx_timer_t *Timer)
{
    // Disable and free the memory for other usages
    switch (Dev)
    {
    case TIMERS::TIMER0:
        nrfx_timer_disable(Timer);
        _IsTimer1Initialized = false;
        break;

    case TIMERS::TIMER1:
        nrfx_timer_disable(Timer);
        _IsTimer2Initialized = false;
        break;

    case TIMERS::SAADC_TIMER:
        nrfx_timer_disable(Timer);
        _IsTimer3Initialized = false;
        break;
    }

    // Free memory and exit
    k_free(Timer);
    return;
}