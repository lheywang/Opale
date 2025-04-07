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
#include "init/init.h"
#include "config.h"
#include "drivers/teseo/teseo_struct.h"

// External libs
#include "drivers/teseo/teseo_gnss_structs.h"
#include "drivers/teseo/NMEA_parser.h"

/* -----------------------------------------------------------------
 * MODULES SETTINGS
 * -----------------------------------------------------------------
 */

/* -----------------------------------------------------------------
 * CONSTANTS
 * -----------------------------------------------------------------
 */
#define TESEO_OK 0
#define TESEO_FAIL -1

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

    /**
     * @brief   Return the UTC time, sent from the GPS satellites.
     *
     * @param   date    A pointer to a date structure to be filled
     *
     * @return  Status code
     * @retval   0      Sucess
     * @retval  -1      Failed to send the command to the TESEO module
     * @retval  -2      Received message was not properly formed.
     * @retval  -3      Failed to parse the message
     */
    uint8_t getUTCTime(GPGGA_Info_t *const date);

    /**
     * @brief   Enable the 1 PPS (Pulse per second) output on the module
     *
     * @return  Status code
     * @retval   0      Sucess
     * @retval  -1      Failed to send the command to the TESEO module
     * @retval  -2      Received message was not properly formed.
     * @retval  -3      Module returned an error
     */
    uint8_t enablePPS();

    /**
     * @brief   Disable the 1 PPS (Pulse per second) output on the module
     *
     * @return  Status code
     * @retval   0      Sucess
     * @retval  -1      Failed to send the command to the TESEO module
     * @retval  -2      Received message was not properly formed.
     * @retval  -3      Module returned an error
     */
    uint8_t disablePPS();

    /**
     * @brief   Enable the odometer measure (distance moved).abs
     *
     * @warning If re-called, the measure will be discarded.
     *
     * @return  Status code
     * @retval   0      Sucess
     * @retval  -1      Failed to send the command to the TESEO module
     * @retval  -2      Received message was not properly formed.
     * @retval  -3      Module returned an error
     */
    uint8_t enableOdometer();

    /**
     * @brief   Disable the odometer measure (distance moved).abs
     *
     * @warning If re-called, the measure will be discarded.
     *
     * @return  Status code
     * @retval   0      Sucess
     * @retval  -1      Failed to send the command to the TESEO module
     * @retval  -2      Received message was not properly formed.
     * @retval  -3      Module returned an error
     */
    uint8_t disableOdometer();

    /**
     * @brief   Configure the GPS used for the positionning
     *
     * @param GPS       Enable the GPS system (non military mode)
     * @param GLONASS   Enable the GLONASS system
     * @param GALILEO   Enable the GALILEO system
     * @param QZSS      Enable the QZSS system
     * @param BEIDOU    Enable the BEIDOU system
     *
     * @return  Status code
     * @retval  0
     * @retval  -1
     */
    uint8_t selectConstellations(bool GPS,
                                 bool GLONASS,
                                 bool GALILEO,
                                 bool QZSS,
                                 bool BEIDOU);

    /**
     * @brief   Return the position of the device on the eart.
     *
     * @param pos   Pointer to GLLPosition struct to be filled.
     *
     * @return  Status code
     * @retval  0
     * @retval  -1
     */
    uint8_t getPosition(GNS_Info_t *const pos);

    /**
     * @brief   Return the satellites used for positionning the device.
     *
     * @warning The memory for the satelittes structs is in any case allocated dynamically.
     *          Providing any buffer will end up on a RAM loss.
     *
     * @param   sat     Pointer to a list of structs. See @ref warning.
     * @param   len     Number of structures allocated.
     *
     * @return  Status code
     * @retval  0
     * @retval  -1
     */
    uint8_t GetSatelites(struct Satellite *const sat,
                         uint8_t *const len);

    /**
     * @brief   Get the Ground Speed
     *
     * @param   vtg     Pointer to a VTGSpeed struct to be filled.
     *
     * @return  Status code
     * @retval  0
     * @retval  -1
     */
    uint8_t GetGroundSpeed(struct VTG_Info_t *const vtg);

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
    uint8_t write(const uint8_t *cmd, uint16_t len);

    static void ISR_RX(const struct device *dev,
                       struct uart_event *evt,
                       void *user_data);

    /* -----------------------------------------------------------------
     * PRIVATE VARIABLES
     * -----------------------------------------------------------------
     */

    // Device pointers
    const device *bus;
    bool device_openned;

    // Bus buffers and status
    volatile char buf[256];
    volatile bool IO_DataReady;

    // Parser structures
    static GNSSParser_Data_t GNSSParser_Data;
    static int gnss_feature;
};