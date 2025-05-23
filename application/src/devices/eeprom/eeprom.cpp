/** ================================================================
 * @file    application/src/devices/eeprom.cpp
 *
 * @brief   eeprom.cpp implement low level eeprom handling procedures.
 *
 * @date    23-02-2025
 *
 * @version 1.0.0
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 *  ================================================================
 */

/* -----------------------------------------------------------------
 * INCLUDING LIBS
 * -----------------------------------------------------------------
 */

// Zephyr
#include <zephyr/drivers/spi.h>

// Local libs
#include "devices/eeprom.h"
#include "init/init.hpp"

/* -----------------------------------------------------------------
 * FUNCTIONS TO COMMAND AN EEPROM
 * -----------------------------------------------------------------
 */

int eeprom::GetNextAddress(MemoryIO *const Command,
                           const EEPROM_RW ReadOrWrite)
{

    // Initializing a tab if none are existing for now...
    static uint32_t CurrentAddresses[EEPROM_NB] = {0};
    static uint8_t CurrentMemory = 0;

    // Clearing the MemoryIO struct...
    memset(Command, 0, sizeof(MemoryIO));

    uint32_t FutureAddress = CurrentAddresses[CurrentMemory] +
                             (EEPROM_PAGE_SIZE * EEPROM_PAGE_PER_SAMPLE);

    // Check if we reached the end of the EEPROM
    if (FutureAddress >= EEPROM_MAX_ADDRESS)
    {
        // Check if we have an additionnal EEPROM available ?
        uint8_t FutureEeprom = CurrentMemory + 1;
        if (FutureEeprom > EEPROM_NB)
            return -1;

        // Compute the settings...
        CurrentMemory = FutureEeprom;
        FutureAddress = CurrentAddresses[CurrentMemory] +
                        (EEPROM_PAGE_SIZE * EEPROM_PAGE_PER_SAMPLE);
    }

    // Storring values into the output struct
    Command->address = CurrentAddresses[CurrentMemory];
    Command->eeprom_id = CurrentMemory;
    Command->RWn = ReadOrWrite;

    // Increment counter
    CurrentAddresses[CurrentMemory] = CurrentAddresses[CurrentMemory] +
                                      (EEPROM_PAGE_SIZE * EEPROM_PAGE_PER_SAMPLE);

    // returning
    return 0;
}

int eeprom::IO(const struct spi_dt_spec *Target[],
               const MemoryIO Command)
{
    return 0;
}