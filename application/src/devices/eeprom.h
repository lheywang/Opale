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
   #define EEPROM_PAGE_SIZE         64
   #define EEPROM_PAGE_PER_SAMPLE   2

    /* -----------------------------------------------------------------
    * SECURITY WITH THE OBJECTS SIZE
    * -----------------------------------------------------------------
    */
   #define EVEN_STRUCT_SIZE (uint8_t)sizeof(EvenPage)
   #define ODD_STRUCT_SIZE (uint8_t)sizeof(OddPage)

    // The preprocessor will check for correcty defined structs.
    #if sizeof(uint8_t[EEPROM_PAGE_PER_SAMPLE * EEPROM_PAGE_SIZE]) != sizeof(uint8_t[EEPROM_PAGE_SIZE])
        #error "Invalid data structure detected. Please ensure your custom \
                Data payload is correctly padded or correctly defined. \
                Data MUST be 64 bytes length."
    #endif

    /* -----------------------------------------------------------------
    * Defining data structure
    * -----------------------------------------------------------------
    */

    /*
     * We're using two structs here to store all of the measured data.
     * Both are complementary and are used together, each one on two consecutive pages.
     * 
     * TO DO : 
     * - Check padding of the data to fit on a single page.
     */

    typedef struct {
        // Global data
        uint32_t timestamp;

        // Barometer parameters
        int32_t temperature;
        int32_t pressure;

        // Bosch IMU Sensor data
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Acceleration;
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Gravity;
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Magnetic;
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } AngularSpeed;
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Angle;
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Gyroscope;
        struct {
            uint16_t a;
            uint16_t b;
            uint16_t c;
            uint16_t d;
        } Quaternion;

        uint8_t __padding[8];
    } EvenPage;

    typedef struct {

        // ST II2SDLPT 1
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Acceleration1

        // ST II2SDLPT 2
        struct {
            uint16_t X;
            uint16_t Y;
            uint16_t Z;
        } Acceleration2;

        // GPS DATA
        uint16_t GroundSpeed;
        uint16_t MagVar;

        int32_t Lattitude;
        int32_t Longitude;

        uint32_t TimestampLastPing;

        uint16_t TrackGood;


        uint16_t CRC16;

        uint8_t __padding[32];
    } OddPage;

    /* -----------------------------------------------------------------
    * Defining IO structure
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
            EvenPage Data1;
            OddPage Data2;

        } Data;
    } MemoryIO;

    /* -----------------------------------------------------------------
    * FUNCTIONS TO COMMAND AN RGB LED
    * -----------------------------------------------------------------
    */

#endif

