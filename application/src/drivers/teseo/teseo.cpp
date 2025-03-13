/** ================================================================
 * @file    application/src/drivers/TESEO/teseo.cpp
 *
 * @brief   Implrement teseo base class, and it's action
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

#include "teseo.h"

TESEO::TESEO()
{
}
TESEO::~TESEO()
{
}

/* -----------------------------------------------------------------
 * FUNCTIONS
 * -----------------------------------------------------------------
 */
uint8_t TESEO::getUTCTime(struct UTCTime *const date)
{
}
uint8_t TESEO::enablePPS()
{
}
uint8_t TESEO::disablePPS()
{
}
uint8_t TESEO::enableOdometer()
{
}
uint8_t TESEO::selectConstellations(bool GPS,
                                    bool GLONASS,
                                    bool GALILEO,
                                    bool QZSS,
                                    bool BEIDOU)
{
}
uint8_t TESEO::getPosition(struct GLLPosition *const pos)
{
}
uint8_t TESEO::GetSatelites(struct Satellite *const sat,
                            uint8_t *const len)
{
}
uint8_t TESEO::GetGroundSpeed(struct VTGSpeed *const vtg)
{
}

uint8_t TESEO::configureAGPS()
{
}
uint8_t TESEO::getAGPSStatus(bool *const status)
{
}
uint8_t TESEO::configurePPS()
{
}
uint8_t TESEO::setFreqRange()
{
}
uint8_t TESEO::setLocalOscillator()
{
}
uint8_t TESEO::resetEphemeris()
{
}

/* -----------------------------------------------------------------
 * BUS IO
 * -----------------------------------------------------------------
 */
uint8_t TESEO::write(char *const cmd, uint16_t len)
{
}

static void TESEO::ISR_RX(const struct device *dev,
                          struct uart_event *evt,
                          void *user_data)
{
}
