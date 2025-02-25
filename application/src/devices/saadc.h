/** ================================================================
 * @file    application/src/devices/saadc.h
 *
 * @brief   saadc.h is a bit different, since we exploit the ADC here
 *          but via the nrfx drivers rathers than the zephyrs one.
 *          This gave us a finer control over the measures, but is 
 *          de facto less portable.
 *
 * @date    25-02-2025
 *
 * @version 1.0.0
 * 
 * @author  l.heywang (leonard.heywang@proton.me)
 * 
 *  ================================================================
 */

// Include once
#ifndef DEF_SAADC
    #define DEF_SAADC

    /* -----------------------------------------------------------------
    * INCLUDING LIBS
    * -----------------------------------------------------------------
    */

    // nRFX
    #include <nrfx_saadc.h>
    #include <nrfx_timer.h>
    #include <nrfx.h>
    
    // Libs
    #include "../init/init.h"

    /* -----------------------------------------------------------------
    * FETCHING NODE PARAMETERS
    * -----------------------------------------------------------------
    */

    // Sample rate
    #define SAADC_SAMPLE_INTERVAL_US    1000

    // Buffer settings
    #define SAADC_BUFFER_SIZE           1000

    // Analog inputs settings
    #define SAADC_INPUT_PIN NRF_SAADC_INPUT_AIN0

    static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(SAADC_INPUT_PIN, 0);

    /* -----------------------------------------------------------------
    * STORAGES BUFFERS
    * -----------------------------------------------------------------
    */
    static uint16_t saadc_buffer[2][SAADC_BUFFER_SIZE] = {0};
    static uint32_t saadc_buffer_index = 0;

    /* -----------------------------------------------------------------
    * FUNCTIONS TO COMMAND THE SAADC
    * -----------------------------------------------------------------
    */

    /**
     * @brief   This function configure the SAADC integrated into the 
     *          chip for our usage. This require a timer that is going
     *          to be used to trigger regularely the start signal. 
     * 
     * @param   Target  The timer instance that is going to be used 
     *                  to trigger the measures.
     * 
     * @warning     Since this function will enable interrupts, it's 
     *              recommended to call it in the last of the init 
     *              procedure.
     * 
     * @return  0   Configured successfully the SAADC
     * @return -1   Failed to configure the timer
     * @return -2   Failed to configure the SAADC
     * @return -3   Failed to configure the SAADC gain
     * @return -4   Failed to configure the advanced behavior of the SAADC
     * @return -5   Failed to configure the first buffer of the SAADC
     * @return -6   Failed to configure the second buffer of the SAADC
     * @return -7   Failed to initialize the SAADC to be ready to sample
     */
    int SAADC_Configure(nrfx_timer_t *Target_Timer);

    /**
     * @brief   This is the main even handler for the SAADC interrupt !
     * 
     * @param   p_event A pointer to an interrup context
     * 
     * @return  None
     */
    static void saadc_event_handler(nrfx_saadc_evt_t const * p_event);

    

#endif

