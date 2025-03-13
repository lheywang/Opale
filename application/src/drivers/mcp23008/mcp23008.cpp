/** ================================================================
 * @file    application/src/drivers/mcp23008/mcp23008.cpp
 *
 * @brief   Implement mcp23088 HAL
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */
#include "mcp23008.h"

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>

// Local libs
#include "../../init/init.h"
#include "../../config.h"

// STD
#include <cstdint>

/* -----------------------------------------------------------------
 * LOGGER CONFIG
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(mcp23008, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * STATIC VARIABLES
 * -----------------------------------------------------------------
 */

// Handle the state of the GPIOs to be able to use them from different threads
static bool _GPIOS[8] = {false,
                         false,
                         false,
                         false,
                         false,
                         false,
                         false,
                         false};

// Handle definition of the I2C device
static i2c_dt_spec *_dev = nullptr;
static bool _IsDevOpenned = false;

/* -----------------------------------------------------------------
 * REGISTERS
 * -----------------------------------------------------------------
 */
constexpr int IODIR = 0x00;
constexpr int IPOL = 0x01;
constexpr int GPINTEN = 0x02;
constexpr int DEFVAL = 0x03;
constexpr int INTCON = 0x04;
constexpr int IOCON = 0x05;
constexpr int GPPU = 0x06;
constexpr int INTF = 0x07;
constexpr int INTCAP = 0x08;
constexpr int GPIO = 0x09;
constexpr int OLAT = 0x0A;

/* -----------------------------------------------------------------
 * CONSTRUCTORS AND DESTRUCTORS
 * -----------------------------------------------------------------
 */
MCP23008::MCP23008(MCP23008_GPIOS pin)
{
    // First, check if the device is openned and available
    if (_IsDevOpenned == false)
    {
        _dev = (i2c_dt_spec *)INIT_GetAnI2C(I2CS::EXPANDER);
        if (_dev == nullptr)
        {
            LOG_ERR("Failed to get the I2C device. Aborting...");
            return;
        }

        // Set the device as openned
        _IsDevOpenned = true;

        // Then, initialize some settings about IOs.
        this->write(IOCON, 0x00, 1);
        this->write(GPINTEN, 0x00, 1);
    }

    // If available, check if the pin is used or not.
    if (_GPIOS[(int)pin] != false)
    {
        LOG_ERR("Pin is already used. Aborting...");
        return;
    }

    // Locking further calls
    this->PinIsAvailable = true;
    _GPIOS[(int)pin] = true;

    // Storing the pin
    this->PinID = (uint8_t)pin;

    // Exiting
    return;
}

MCP23008::~MCP23008()
{
    // First, set the pin to unused :
    // I2C call

    // Then, free the pin
    _GPIOS[this->PinID] = false;

    // Check if the device is used anymore ?
    bool status = false;
    for (uint8_t k = 0; k < 8; k++)
    {
        if (_GPIOS[k] == true)
        {
            status = true;
            break;
        }
    }

    if (status == true)
    {
        // Free the device
        INIT_FreeAnI2C(I2CS::BAROMETER, _dev);
        _dev = nullptr;
    }
}

/* -----------------------------------------------------------------
 * FUNCTIONS
 * -----------------------------------------------------------------
 */

uint8_t MCP23008::enablePullUp()
{
    uint8_t buf = 0;
    int err = this->read(GPPU, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual GPPU register (enable PullUp)");
        return -1;
    }

    buf |= (1 << this->PinID);

    err = this->write(GPPU, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to write the GPPU register (enable PullUp)");
    }

    return 0;
}

uint8_t MCP23008::disablePullUp()
{
    uint8_t buf = 0;
    int err = this->read(GPPU, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual GPPU register (disable PullUp)");
        return -1;
    }

    buf &= ~(1 << this->PinID);

    err = this->write(GPPU, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to write the GPPU register (disable PullUp)");
    }

    return 0;
}

uint8_t MCP23008::readPin()
{
    uint8_t buf = 0;
    int err = this->read(GPIO, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual GPIO register (Read Pin)");
        return -1;
    }

    return (buf & (1 << this->PinID));
}

uint8_t MCP23008::writePin(uint8_t value)
{
    uint8_t buf = 0;
    int err = this->read(OLAT, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual OLAT register (Write Pin)");
        return -1;
    }

    if (value)
    {
        buf |= (1 << this->PinID);
    }
    else
    {
        buf &= ~(1 << this->PinID);
    }

    err = this->write(GPPU, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to write the OLAT register (Write Pin)");
    }

    return 0;
}

uint8_t MCP23008::setInputPolarity(const uint8_t value)
{
    uint8_t buf = 0;
    int err = this->read(IPOL, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual IPOL register (enable PullUp)");
        return -1;
    }

    if (value)
    {
        buf |= (1 << this->PinID);
    }
    else
    {
        buf &= ~(1 << this->PinID);
    }

    err = this->write(GPPU, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to write the IPOL register (enable PullUp)");
    }

    return 0;
}

uint8_t MCP23008::setPinAsInput()
{
    uint8_t buf = 0;
    int err = this->read(IODIR, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual IODIR register (Set Input)");
        return -1;
    }

    buf |= (1 << this->PinID);

    err = this->write(IODIR, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to write the IODIR register (Set Input)");
    }

    return 0;
}

uint8_t MCP23008::setPinAsOutput()
{
    uint8_t buf = 0;
    int err = this->read(IODIR, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to fetch the actual IODIR register (Set Output)");
        return -1;
    }

    buf &= ~(1 << this->PinID);

    err = this->write(IODIR, &buf, 1);
    if (err != 0)
    {
        LOG_ERR("Failed to write the IODIR register (Set Output)");
    }

    return 0;
}

/* -----------------------------------------------------------------
 * PRIVATE
 * -----------------------------------------------------------------
 */

uint8_t MCP23008::write(uint8_t reg, uint8_t *data, uint8_t wlen)
{
    if (_IsDevOpenned == false)
    {
        LOG_ERR("Device is not openned. Cancelled write...");
        return -1;
    }

    uint8_t buf[wlen + 1] = {0};
    buf[0] = (uint8_t)reg & 0xFF;

    for (uint8_t k = 0; k < wlen - 1; k++)
    {
        buf[k + 1] = data[k];
    }

    return i2c_write_dt(_dev, data, wlen + 1);
}
uint8_t MCP23008::read(uint8_t reg, uint8_t *data, uint8_t rlen)
{
    if (_IsDevOpenned == false)
    {
        LOG_ERR("Device is not openned. Cancelled read...");
        return -1;
    }

    return i2c_write_read_dt(_dev, &reg, 1, data, rlen);
}