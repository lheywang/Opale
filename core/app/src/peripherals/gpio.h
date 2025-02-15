/**
 * ---------------------------------------------------------------
 * 
 * @file    core/app/src/peripherals/gpio.c
 *
 * @brief   Expose standard functions for the GPIO usage on the 
 *          nRF5340 SoC.
 *
 * @author  l.heywang (leonard.heywang@proton.me)
 *
 * @date    15/02/2025
 *
 * @version 1.0.0
 *
 * ---------------------------------------------------------------
 */

// STDLIB 
#include <stdint.h>

// ==============================================================
// Enums
// ==============================================================

/**
 * @brief   Assign value to direction for GPIO.
 *          Used to handle error and software protections !
 * 
 */
typedef enum {
    GPIO_INPUT,
    GPIO_OUTPUT
} GPIOMode;

// ==============================================================
// Structs
// ==============================================================

/**
 * @brief   Define the GPIO struct that is used for standard
            operations on the nRF5340 GPIOs...
 * 
 */
typedef struct {
    uint8_t Port;
    uint8_t Pin;
    GPIOMode Mode;
} GPIO;

// ==============================================================
// Functions declaration
// ==============================================================

/**
 * @brief   Open and create a new GPIO struct to handle standard IO
 * 
 * @param   Port    Port number (uint8_t)
 * @param   Pin     Pin number (uint8_t
 * @param   Mode    Mode GPIOMode Enum.
 *
 * @return  GPIO    A GPIO struct 
 */
GPIO* GPIO_Open(uint8_t Port, uint8_t Pin, GPIOMode Mode);

/**
 * @brief   Close and delete the GPIO struct.
 * 
 * @param   Target  Pointer to a GPIO struct
 *
 * @return  0 :     Structure correctly deleted
                    and memory was freed
 */
int GPIO_Close(GPIO *Target);

/**
 * @brief   Write a value to a GPIO.
 * 
 * @param   Target  Pointer to a GPIO struct
 * @param   Value   The value to be written (0 or 1)
 *
 * @return  0 :     Value was written
 * @return -1 :     Pin is configured as input. Cannot write.   
 */
int GPIO_Write(GPIO *Target, int Value);

/**
 * @brief   Read the value of the GPIO (Input or Output,
 *          depending on it's mode)
 * 
 * @param   Target  Pointer to a GPIO struct
 * @param   Value   Pointer to an integer where the value
 *                  will be stored.
 * 
 * @return  0 :     Value was read
 */
int GPIO_Read(GPIO *Target, int *Value);

/**
 * @brief   Toggle the GPIO if configured as output
 * 
 * @param   Target  Pointer to a GPIO struct
 *
 * @return  0 :     Pin was toggled
 * @return -1 :     Cannot toggle pin, pin is configured as input. 
 */
int GPIO_Toggle(GPIO *Target);