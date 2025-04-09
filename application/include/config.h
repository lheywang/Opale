/** ================================================================
 * @file    application/src/config.h
 *
 * @brief   This file configure some small settings to the project
 *          level.
 *
 * @date    20-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

#ifndef DEF_CONFIG
#define DEF_CONFIG

/*
 * -----------------------------------------------------------------
 * INCLUDING LINS
 * -----------------------------------------------------------------
 */
#include <zephyr/logging/log.h>

/*
 * -----------------------------------------------------------------
 * DEFINES
 * -----------------------------------------------------------------
 */
/*
 * Configure the project log level for all files.
 * This is an easier way to do rather than change all files
 *
 * Available options are :
 *  - LOG_LEVEL_DBG (All)
 *  - LOG_LEVEL_INF (Near all)
 *  - LOG_LEVEL_WRN (Only warnings)
 *  - LOG_LEVEL_ERR (Errors only)
 */
#define PROJECT_LOG_LEVEL LOG_LEVEL_DBG

#endif /* DEF_CONFIG*/