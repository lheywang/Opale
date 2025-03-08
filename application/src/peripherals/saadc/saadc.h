/** ================================================================
 * @file    application/src/peripherals/saadc/saadc.h
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
    #include "../../init/init.h"

    /* -----------------------------------------------------------------
    * FETCHING NODE PARAMETERS
    * -----------------------------------------------------------------
    */

    // Sample rate of the ADC
    #define SAADC_CHANNEL_SAMPLE_RATE   1000 // us
    
    // Buffer settings
    #define SAADC_BUFFER_SIZE           2500 // 2.5 seconds

    // Analog inputs settings
    #define SAADC_INPUT_COUNT           8

    // Analog inputs channels
    static nrfx_saadc_channel_t channels[SAADC_INPUT_COUNT] = {     NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN7, 0),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN6, 1),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN1, 2),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 3),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN4, 4),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN3, 5),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN2, 6),
                                                                    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN5, 7)};
                                            
    /*
     * The defined channels here may seem to be in disorder, but this is intentional.
     * Actully, this match our PCB layout and our needs.
     */

    // Internal defines
    #define SAADC_SAMPLE_INTERVAL_US    SAADC_INPUT_COUNT * SAADC_CHANNEL_SAMPLE_RATE

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
     * @brief   This function is able to interrupt the ADC w
     * 
     * @param Target_Timer 
     * @return int 
     */
    int SAADC_Stop(nrfx_timer_t *Target_Timer);

    /**
     * @brief   This is the main even handler for the SAADC interrupt !
     * 
     * @param   p_event A pointer to an interrup context
     * 
     * @return  None
     */
    static void saadc_event_handler(nrfx_saadc_evt_t const * p_event);

    

#endif

