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
#include "../../init/init.h"
#include "../../config.h"

// STD
#include <cstdint>

/* -----------------------------------------------------------------
 * PRIVATE DEFINES
 * -----------------------------------------------------------------
 */
// Let's define standard commands
constexpr char *_NMEA_PPSEN = (char *)"$PSTMCFGPPSGEN,1,0,0,0*_\r\n";
constexpr char *_NMEA_PPSOF = (char *)"$PSTMCFGPPSGEN,0,0,0,0*_\r\n";
constexpr char *_NMEA_ODOEN = (char *)"$PSTMCFGODO,1,0,65535*_\r\n";
constexpr char *_NMEA_ODOOF = (char *)"$PSTMCFGODO,0,0,65535*_\r\n";

// Define NMEA command delimiters
constexpr char CMD_Addr = '$';
constexpr char CMD_Chkm = '*';
constexpr char Chksm_placeholder = '_';

/* -----------------------------------------------------------------
 * PRIVATE FUNCTIONS
 * -----------------------------------------------------------------
 */

/*
 * Compute the checksum.
 *
 * This function shall be executed at compile time rather than on the nRF !
 */
const char *_TESEO_CHKSM(const char *input, uint8_t len)
{
    // Check that the input is well formed
    if (char2int(input[0]) != char2int('$'))
    {
        return (char *)' ';
    }

    char *placeholder = strchr(input, CMD_Chkm);
    if (placeholder < (input + len))
    {
        return (char *)' ';
    }

    if (strchr(input, Chksm_placeholder) < (input + len))
    {
        return (char *)' ';
    }

    // Iterate over the string to compute the checksum
    int n = 0;
    uint8_t checksum = 0;

    while ((char2int(input[n] != char2int('*'))) || (n < len))
    {
        checksum ^= input[n];
        n += 1;
    }

    // Inserting the computed value into the char
    *placeholder = checksum;
    return input;
}

/* -----------------------------------------------------------------
 * PRIVATE COMMANDS DEFINES
 * -----------------------------------------------------------------
 */
// Theses values are computed by GCC at the compile time
const char *NMEA_PPSEN = _TESEO_CHKSM(_NMEA_PPSEN, sizeof(_NMEA_PPSEN));
const char *NMEA_PPSOF = _TESEO_CHKSM(_NMEA_PPSOF, sizeof(_NMEA_PPSOF));
const char *NMEA_ODOEN = _TESEO_CHKSM(_NMEA_ODOEN, sizeof(_NMEA_ODOEN));
const char *NMEA_ODOOF = _TESEO_CHKSM(_NMEA_ODOOF, sizeof(_NMEA_ODOOF));

// Commands
const char *NMEA_PSTIM = (char *)"$PSTMTIM\r\n";

// Return values
const char *PSTMCFGPPSGENOK = (char *)"$PSTMCFGPPSGENOK";
const char *PSTMCFGODOOK = (char *)"$PSTMCFGODOOK";

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
uint8_t TESEO::getUTCTime(GPGGA_Info_t *const date)
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_PSTIM, sizeof(NMEA_PSTIM));
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Start the parsing of the message
    int status = GNSS_PARSER_ParseMsg(&this->GNSSParser_Data, eNMEAMsg::GPGGA, (uint8_t *)this->buf);
    if (status == PARSE_FAIL)
    {
        return -3;
    }

    // If sucessfull, we copy the output of into the custom allocated struct
    memcpy((void *)date, (void *)&this->GNSSParser_Data.gpgga_data, sizeof(GPGGA_Info_t));

    // Exiting the function !
    return 0;
}

uint8_t TESEO::enablePPS()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_PPSEN, sizeof(NMEA_PPSEN));
    if (ret != 0)
    {
        return -1;
    }

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGPPSGENOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGPPSGENOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGPPSGENOK) != 0)
    {
        return -3;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::disablePPS()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_PPSOF, sizeof(NMEA_PPSOF));
    if (ret != 0)
    {
        return -1;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGPPSGENOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGPPSGENOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGPPSGENOK) != 0)
    {
        return -3;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::enableOdometer()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_ODOEN, sizeof(NMEA_ODOEN));
    if (ret != 0)
    {
        return -1;
    }

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGODOOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGODOOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGODOOK) != 0)
    {
        return -2;
    }

    // Exit the function
    return 0;
}

uint8_t TESEO::disableOdometer()
{
    // First, write the command to the TESEO module
    int ret = this->write((uint8_t *)NMEA_ODOOF, sizeof(NMEA_ODOOF));
    if (ret != 0)
    {
        return -1;
    }

    // Checking the sanity of the message
    // And returning if something when wrong...
    int check = GNSS_PARSER_CheckSanity((uint8_t *)this->buf, sizeof(this->buf));
    if (check != GNSS_PARSER_ERROR)
    {
        return -2;
    }

    // Wait for the data to be ready by yielding the data (= exiting the task to let the remaining run...)
    // This variable shall be configured to true by the ISR !
    while (this->IO_DataReady == false)
    {
        k_yield();
    }
    this->IO_DataReady = false;

    // Checking if the message is correctly returned
    // Thus, we copy the N first characters
    char tmp[sizeof(PSTMCFGODOOK)] = {'\0'};
    memcpy((void *)tmp, (void *)&this->buf, sizeof(PSTMCFGODOOK));

    // Compare the two strings
    if (strcmp(tmp, PSTMCFGODOOK) != 0)
    {
        return -2;
    }

    // Exit the function
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
uint8_t TESEO::write(const uint8_t *cmd, uint16_t len)
{

    // Ask for Zephyr to transmit the bytes
    int ret = uart_tx(this->bus, cmd, len, SYS_FOREVER_US);
    if (ret)
    {
        return -1;
    }

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
