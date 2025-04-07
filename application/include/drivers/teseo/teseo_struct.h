/** ================================================================
 * @file    application/src/drivers/TESEO/struct.h
 *
 * @brief   Define teseo storage struct for unsuported types
 *
 * @date    07-04-2025
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

#ifndef _DEF_TESEO_STRUCTS
#define _DEF_TESEO_STRUCTS

/* -----------------------------------------------------------------
 * STRUCTS
 * -----------------------------------------------------------------
 */

 struct VTG_Info_t {
    double TrackDifReal;
    double TrackDifMag;
    double KnotsSpeed;
    double KMSpeed;
    char Mode;
 };

#endif /* _DEF_TESEO_STRUCTS */