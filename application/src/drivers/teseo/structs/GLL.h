/** ================================================================
 * @file    application/src/drivers/TESEO/structs/GLL.h
 *
 * @brief   Define GLL position storage struct
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */
#ifndef DEF_GLL
#define DEF_GLL

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */

// STD
#include <cstdint>

// Libs
#include "UTC.h"

/* -----------------------------------------------------------------
 * AUX STORAGE STRUCT
 * -----------------------------------------------------------------
 */
struct Direction
{
    char Direction;
    uint8_t Degree;
    double Minutes;
};

/* -----------------------------------------------------------------
 * MAIN STORAGE STRUCT
 * -----------------------------------------------------------------
 */
struct GLLPosition
{
    UTCTime Timestamp;
    Direction Lattitude;
    Direction Longitude;
    uint8_t GPSPrecision;
    uint8_t NumberOfSatellites;
    double Dilution;
    uint16_t Altitude;
};

#endif /* DEF_GLL */