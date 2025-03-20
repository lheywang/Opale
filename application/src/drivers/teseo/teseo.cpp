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

/* -----------------------------------------------------------------
 * Libs
 * -----------------------------------------------------------------
 */
// Associated header
#include "teseo.h"

// Zephyr
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>

// External libs
#include "LibGNSS/gnss_parser.h"
#include "LibNMEA/NMEA_parser.h"
#include "LibGNSS1A1/gnss1a1_gnss.h"
#include "teseo_liv3f_conf.h"

// Internal libs
#include "structs/GLL.h"
#include "structs/SAT.h"
#include "structs/UTC.h"
#include "structs/VTG.h"

#include "../../init/init.h"
#include "../../config.h"

/* -----------------------------------------------------------------
 * LOGGER MODULE
 * -----------------------------------------------------------------
 */
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(Teseo, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
 * CONSTRUCTORS AND DESTRUCTORS
 * -----------------------------------------------------------------
 */

TESEO::TESEO()
{
    // Open the UART bus
    this->bus = INIT_GetAnUART(UARTS::GPS);
    if (this->bus == nullptr)
    {
        this->device_openned = false;
        return;
    }
    this->device_openned = true;

    // GNSS1A1 init
    int ret = 0;
    ret = GNSS1A1_GNSS_Init(GNSS1A1_TESEO_LIV3F);
    if (ret != 0)
    {
        LOG_ERR("Failed to initialize GNSS1A1. Aborting...");
        return;
    }

    // Initializing internal variables
    this->gnss_feature = 0x0;
    this->IO_DataReady = false;

    // Initializing parser
    ret = GNSS_PARSER_Init(&this->GNSSParser_Data);
    if (ret != 0)
    {
        LOG_ERR("Failed to initialize GNSS Parser. Aborting...");
        return;
    }

    // Then, register the bus communication ISR (and data)
    ret = uart_callback_set(this->bus,
                            TESEO::ISR_RX,
                            (void *)this);
    if (ret != 0)
    {
        LOG_ERR("Failed to set the RX callback ISR !");
        return;
    }

    // Enabling UART reception
    ret = uart_rx_enable(this->bus,
                         (uint8_t *)this->buf,
                         sizeof(this->buf),
                         2048);
    // 2048 us without no new data.
    // This correspond to 2 char at 9600 baud.
    // This delay is used to differentiate two messages from the others.

    if (ret != 0)
    {
        LOG_ERR("Failed to enable the UART RX !");
        return;
    }

    // Device is initialized, exiting !
    return;
}
TESEO::~TESEO()
{
    // Closing UART HAL !
    INIT_FreeAnUART(UARTS::GPS, this->bus);
}

/* -----------------------------------------------------------------
 * FUNCTIONS
 * -----------------------------------------------------------------
 */

/*
 * For each function, the use principle differ a little bit
 * from the original ST example.
 *
 * In the example, they used a separate thread, that respond
 * to user commands over UART and send them to the GPS.
 * The GPS thread is thus only a listener on this bus, where,
 * in our case we're also writter.
 */
uint8_t TESEO::getUTCTime(struct UTCTime *const date)
{
    return 0;
}
uint8_t TESEO::enablePPS()
{
    return 0;
}
uint8_t TESEO::disablePPS()
{
    return 0;
}
uint8_t TESEO::enableOdometer()
{
    return 0;
}
uint8_t TESEO::selectConstellations(bool GPS,
                                    bool GLONASS,
                                    bool GALILEO,
                                    bool QZSS,
                                    bool BEIDOU)
{
    return 0;
}
uint8_t TESEO::getPosition(struct GLLPosition *const pos)
{
    return 0;
}
uint8_t TESEO::GetSatelites(struct Satellite *const sat,
                            uint8_t *const len)
{
    return 0;
}
uint8_t TESEO::GetGroundSpeed(struct VTGSpeed *const vtg)
{
    return 0;
}

uint8_t TESEO::configureAGPS()
{
    return 0;
}
uint8_t TESEO::getAGPSStatus(bool *const status)
{
    return 0;
}
uint8_t TESEO::configurePPS()
{
    return 0;
}
uint8_t TESEO::setFreqRange()
{
    return 0;
}
uint8_t TESEO::setLocalOscillator()
{
    return 0;
}
uint8_t TESEO::resetEphemeris()
{
    return 0;
}

/* -----------------------------------------------------------------
 * BUS IO
 * -----------------------------------------------------------------
 */
uint8_t TESEO::write(char *const cmd, uint16_t len)
{
    return 0;
}

void TESEO::ISR_RX(const struct device *dev,
                   struct uart_event *evt,
                   void *user_data)
{
    switch (evt->type)
    {
    case UART_RX_RDY:
    {
        // Configure the variable to be ready !
        bool *DataReady = (bool *)user_data;
        *DataReady = true;
        break;

        /*
         * This variable may be triggered by any error while receiving, for example a cable breakdown.
         * Make sure that the link is reliable enough before attempting any operations on it, or the
         * data may be corrupted.
         *
         */
    }

    default:
    {
        LOG_WRN("Unhandled UART RX Event %d", evt->type);
    }
    }

    return;
}
