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
#define DEF_EEPROM

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/drivers/spi.h>

// Libs
#include "../../init/init.h"

// STDBLIB
#include <assert.h>

/* -----------------------------------------------------------------
 * SOME DEFINES ABOUT THE EEPROM
 * -----------------------------------------------------------------
 */
// Global EEPROM specs
#define EEPROM_MAX_ADDRESS 0x1FFFFF

// Page specs
#define EEPROM_PAGE_SIZE 512
#define EEPROM_PAGE_PER_SAMPLE 0.25 // 4 samples per pages !

// Container specs
#define DATA_BYTE_LEN sizeof(Measure)

/* -----------------------------------------------------------------
 * Defining data structure
 * -----------------------------------------------------------------
 */

/*
 * We're using a single struct, of 128 bytes long that is used to store
 * all measures for a single timestamp.
 *
 * The EEPROM we chose has page of 512 bytes, we can write four measures
 * per page. Since the EEPROM can at most perform write on a single page
 * per command, we can't go on the next page until wrote is done (~5 ms).
 * Thus, is the struct is longer, we'll need to wait, which is not wanted.
 *
 * We may use QSPI, but that's not needed, since we're writting a lot, and
 * the transfer time (140 us @8 MHz or 56 us @20 MHZ) is way smaller than
 * the page write time (~5 ms) (factor of 100 !).
 *
 */

typedef struct
{
    // Global data
    uint32_t timestamp;

    // Barometer parameters
    int32_t temperature;
    int32_t pressure;

    // Bosch IMU Sensor data
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } Acceleration;
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } Gravity;
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } Magnetic;
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } AngularSpeed;
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } Angle;
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } Gyroscope;
    struct
    {
        uint16_t a;
        uint16_t b;
        uint16_t c;
        uint16_t d;
    } Quaternion;

    uint8_t __padding1[8];

    // ST II2SDLPT 1
    struct
    {
        uint16_t X;
        uint16_t Y;
        uint16_t Z;
    } Acceleration1;

    // ST II2SDLPT 2
    struct
    {
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

    uint8_t __padding2[32];
} Measure;

/* -----------------------------------------------------------------
 * Enums
 * -----------------------------------------------------------------
 */
typedef enum
{
    EEPROM_READ,
    EEPROM_WRITE
} EEPROM_RW;

/* -----------------------------------------------------------------
 * Defining IO structure
 * -----------------------------------------------------------------
 */

typedef struct
{
    EEPROM_RW RWn;
    uint8_t eeprom_id;
    uint32_t address;

    union
    {
        // first field is used as "cast" before any IO operation
        // to the EEPROM
        uint8_t Binary[DATA_BYTE_LEN];

        // Add here you're Data io in the format
        Measure Data;

    } Data;
} MemoryIO;

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND AN EEPROM
 * -----------------------------------------------------------------
 */

/**
 * @brief   This function return, on the pointer value the next
 *          write address for the data.
 *
 * @warning Any dummy call will create empty bytes
 *          (and thus, lost EEPROM space)
 *
 * @param   Address     A pointer to a MemoryIO structure
 *                      that will be flushed and filled with
 *                      computed values.
 * @param   ReadOrWrite A bool that defined the IO to be performed.
 *                      Set to True to Write, otherwise Read.
 *
 * @return  0   Value got correctly
 * @return -1   No more address available.
 */
int EEPROM_GetNextAddress(MemoryIO *const Command, const EEPROM_RW ReadOrWrite);

/**
 * @brief   This function perform an IO operation on the eeprom,
 *          following the instructions and parameters passed on
 *          the Command struct.
 *
 * @param   Eeproms     The array of EEPROM available
 * @param   Command     The command struct that define parameters
 *                      for the operation.
 *
 * @return  0   IO completer correctly.
 * @return -1   EEPROM is busy (to recent write operation, within 5ms)
 * @return -2   Invalid Command structure
 */
int EEPROM_IO(const struct spi_dt_spec *Target[], const MemoryIO Command);

#endif /* DEF_EEPROM*/
