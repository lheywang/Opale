/** ================================================================
 * @file    application/src/drivers/TESEO/teseo.h
 *
 * @brief   Define teseo base class, and it's action
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 * @note    This class has been derived and adapted from an original
 *          ST example.
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */
// Zephyr
#include <zephyr/drivers/uart.h>

// STD
#include <cstdint>

// Custom
#include "structs/UTC.h"
#include "structs/GLL.h"
#include "structs/SAT.h"
#include "../../init/init.h"
#include "../../config.h"

/* -----------------------------------------------------------------
 * CLASS
 * -----------------------------------------------------------------
 */
class TESEO
{
public:
    /* -----------------------------------------------------------------
     * CONSTRUCTOR / DESTRUCTORS
     * -----------------------------------------------------------------
     */
    TESEO();
    ~TESEO();

    /* -----------------------------------------------------------------
     * FUNCTIONS
     * -----------------------------------------------------------------
     */
    uint8_t getUTCTime(struct UTCTime *const date);
    uint8_t enablePPS();
    uint8_t disablePPS();
    uint8_t enableOdometer();
    uint8_t selectConstellations(bool GPS,
                                 bool GLONASS,
                                 bool GALILEO,
                                 bool QZSS,
                                 bool BEIDOU);
    uint8_t getPosition(struct GLLPosition *const pos);
    uint8_t GetSatelites(struct Satellite *const sat,
                         uint8_t *const len);
    uint8_t GetGroundSpeed(struct VTGSpeed *const vtg);

private:
    uint8_t configureAGPS();
    uint8_t getAGPSStatus(bool *const status);
    uint8_t configurePPS();
    uint8_t setFreqRange();
    uint8_t setLocalOscillator();
    uint8_t resetEphemeris();

    /* -----------------------------------------------------------------
     * BUS IO
     * -----------------------------------------------------------------
     */
    uint8_t write(char *const cmd, uint16_t len);

    static void ISR_RX(const struct device *dev,
                       struct uart_event *evt,
                       void *user_data);

    /* -----------------------------------------------------------------
     * PRIVATE VARIABLES
     * -----------------------------------------------------------------
     */

    // Device pointers
    const device *bus;

    // Bus buffers and status
    volatile char buf[256];
    volatile bool DataValid;
};