/** ================================================================
 * @file    application/src/drivers/TESEO/structs/VTG.h
 *
 * @brief   Define VTG
 *
 * @date    13-03-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */
#ifndef DEF_VTG
#define DEF_VTG

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
struct VTGSpeed
{
    double RealTrack;
    double MagTrack;
    double Knots;
    double Kmh;
};

#endif /* DEF_VTG */