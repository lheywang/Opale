/** ================================================================
 * @file    application/src/drivers/TESEO/structs/SAT.h
 *
 * @brief   Define SAT
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */
#ifndef DEF_SAT
#define DEF_SAT

/* -----------------------------------------------------------------
 * LIBS
 * -----------------------------------------------------------------
 */

// STD
#include <cstdint>

/* -----------------------------------------------------------------
 * MAIN STORAGE STRUCT
 * -----------------------------------------------------------------
 */
struct Sattelite
{
    uint8_t ID;
    double Elevation;
    double Azimuth;
    uint8_t SNR;
};

#endif /* DEF_SAT */