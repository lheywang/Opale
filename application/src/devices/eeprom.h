/** ================================================================
 * @file    application/src/devices/eeprom.h
 *
 * @brief   eeprom.h define some high level functions and data structure
 *          to interracts with the EEPROM (M95256)
 *          
 * @date    21-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

// Include once
#ifndef DEF_EEPROM
    #define DEF_RGB

    /* -----------------------------------------------------------------
    * INCLUDING LIBS
    * -----------------------------------------------------------------
    */

    // Zephyr
    #include <zephyr/drivers/spi.h>
    
    // Libs
    #include "../init/init.h"

    /* -----------------------------------------------------------------
    * FETCHING NODE PARAMETERS
    * -----------------------------------------------------------------
    */

    // Period

    /* -----------------------------------------------------------------
    * SOME DEFINES ABOUT THE EEPROM
    * -----------------------------------------------------------------
    */
   #define EEPROM_PAGE_SIZE 64

    /* -----------------------------------------------------------------
    * SECURITY WITH THE OBJECTS SIZE
    * -----------------------------------------------------------------
    */
    // The preprocessor will check for correcty defined structs.
    #if sizeof(uint8_t[63]) != sizeof(uint8_t[EEPROM_PAGE_SIZE])
        #error "Invalid data structure detected. Please ensure your custom \
                Data payload is correctly padded or correctly defined. \
                Data MUST be 64 bytes length."
    #endif

    /* -----------------------------------------------------------------
    * Defining command structure
    * -----------------------------------------------------------------
    */

    typedef struct {
        bool RWn;
        uint16_t address;

        union {
            // first field is used as "cast" before any IO operation 
            // to the EEPROM
            uint8_t Binary[EEPROM_PAGE_SIZE];

            // Add here you're Data io in the format

        } Data;
    } MemoryIO;

    /* -----------------------------------------------------------------
    * FUNCTIONS TO COMMAND AN RGB LED
    * -----------------------------------------------------------------
    */

#endif

