/** ================================================================
 * @file    application/src/drivers/mcp23008/mcp23008.h
 *
 * @brief   Define mcp23088 HAL
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 * @warning Interrupts aren't wired on the PCB, so they're not
 *          implement on the class.
 *
 *  ================================================================
 */

#ifndef MCP23008__INC
#define MCP23008__INC

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */
#include <cstdint>

/* -----------------------------------------------------------------
 * ENUMS
 * -----------------------------------------------------------------
 */
typedef enum
{
    GPIO0,
    GPIO1,
    GPIO2,
    GPIO3,
    GPIO4,
    GPIO5,
    GPIO6,
    GPIO7
} MCP23008_GPIOS;

/* -----------------------------------------------------------------
 * CLASS
 * -----------------------------------------------------------------
 */

/**
 * @brief   This class define A GPIO of the MCP23008. This is done to
 *          separate the differents GPIO from each threads, and enabling
 *          to use each pin as an individual pin.
 *
 */
class MCP23008
{
public:
    /* -----------------------------------------------------------------
     * CONSTRUCTORS AND DESTRUCTORS
     * -----------------------------------------------------------------
     */
    MCP23008(MCP23008_GPIOS pin);
    ~MCP23008();

    /* -----------------------------------------------------------------
     * FUNCTIONS
     * -----------------------------------------------------------------
     */
    // -----------------------------------------------------------------
    /**
     * @brief   Enable the pull up on a specific pin.
     *
     * @return  uint8_t  0 : Sucess
     *                  -1 : Fail
     */
    uint8_t enablePullUp();

    /**
     * @brief   Disable a pull up on a specific pin
     *
     * @return  uint8_t  0 : Sucess
     *                  -1 : Fail
     */
    uint8_t disablePullUp();

    // -----------------------------------------------------------------
    /**
     * @brief   Read a pin
     *
     * @return  uint8_t  >The pin value (or -1 in case of error)
     */
    uint8_t readPin();

    /**
     * @brief   Write a value for a pin
     *
     * @param   value   Value to be written
     *
     * @return  uint8_t  0 : Sucess
     *                  -1 : Fail
     */
    uint8_t writePin(uint8_t value);

    // -----------------------------------------------------------------
    /**
     * @brief Set the Input Polarity
     *
     * @param   value    If set to > 1 : Value is inverted
     *
     * @return  uint8_t  0 : Sucess
     *                  -1 : Fail
     */
    uint8_t setInputPolarity(const uint8_t value);
    /**
     * @brief   Set the pin as input
     *
     * @return  uint8_t  0 : Sucess
     *                  -1 : Fail
     */
    uint8_t setPinAsInput();
    /**
     * @brief   Set the pin as output
     *
     * @return  uint8_t  0 : Sucess
     *                  -1 : Fail
     */
    uint8_t setPinAsOutput();

private:
    bool PinIsAvailable;
    uint8_t PinID;

    uint8_t write(uint8_t reg, uint8_t *data, uint8_t wlen);
    uint8_t read(uint8_t reg, uint8_t *data, uint8_t rlen);
};

#endif /* MCP23008 */