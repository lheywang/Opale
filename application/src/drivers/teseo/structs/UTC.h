/** ================================================================
 * @file    application/src/drivers/TESEO/structs/UTC.h
 *
 * @brief   Define UTC time storage struct
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */
#ifndef DEF_UTC
#define DEF_UTC

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */

// STD
#include <cstdint>

/* -----------------------------------------------------------------
 * STORAGE STRUCT
 * -----------------------------------------------------------------
 */
struct UTCTime
{
    uint16_t year;
    uint8_t month;
    uint8_t day;

    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millis;
};

#endif /* DEF_UTC*/