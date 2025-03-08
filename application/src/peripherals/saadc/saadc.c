/** ================================================================
 * @file    application/src/peripherals/saadc/saadc.c
 *
 * @brief   saadc.c implement some low level functions for the
 *          control of the SAADC (Successive approximation 
 *          analog to digital converter).
 *
 * @date    25-02-2025
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

// nRFX
#include <nrfx.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_dppi.h> // This line must be replaced with nrfx_ppi for nRF52 and less series.

// Libs
#include "../init/init.h"
#include "../config.h"
#include "saadc.h"

// Zephyr
#include <zephyr/logging/log.h>

/* -----------------------------------------------------------------
* LOGGER CONFIG
* -----------------------------------------------------------------
*/
// Identify the module on the LOG Output
LOG_MODULE_REGISTER(SAADC, PROJECT_LOG_LEVEL);

/* -----------------------------------------------------------------
* FUNCTIONS TO COMMAND AN SAADC
* -----------------------------------------------------------------
*/

int SAADC_Configure(nrfx_timer_t *Target_Timer){
    int err = 0;

    // First, configure the timer to default settings
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000);
    err = nrfx_timer_init(Target_Timer, &timer_config, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to initialize the timer for the SAADC: %08x", err);
        return -1;
    }

    // Then, configure the timer for OUR needs :
    uint32_t timer_ticks = nrfx_timer_us_to_ticks(  Target_Timer, 
                                                    SAADC_SAMPLE_INTERVAL_US);
    nrfx_timer_extended_compare(Target_Timer, 
                                NRF_TIMER_CC_CHANNEL0, 
                                timer_ticks, 
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, 
                                false);

    // Initializing the interrupt handler
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
            DT_IRQ(DT_NODELABEL(adc), priority),
            nrfx_isr, nrfx_saadc_irq_handler, 0);

    // Ensure that the ADC is working fine
    err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to initialize the SAADC module: %08x", err);
        return -2;
    }

    // Configure the ADC gain
    for (uint8_t k = 0; k < 8; k++)
        channels[k].channel_config.gain = NRF_SAADC_GAIN1_6;

    // Configuring the channels used
    err = nrfx_saadc_channels_config(channels, 8);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to configure the SAADC gain: %08x", err);
        return -3;
    }

    // Configure the SAADC operation mode (NO OVERSAMPLING, NO BURST, no autostart...)
    nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    err = nrfx_saadc_advanced_mode_set( 255UL,
                                        NRF_SAADC_RESOLUTION_12BIT,
                                        &saadc_adv_config,
                                        saadc_event_handler);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to configure advanced mode of the SAADC: %08x", err);
        return -4;
    }

    // Pass the double buffers to the ADC
    err = nrfx_saadc_buffer_set(saadc_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to set the buffer 0: %08x", err);
        return -5;
    }
    err = nrfx_saadc_buffer_set(saadc_buffer[0], SAADC_BUFFER_SIZE);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to set the buffer 1: %08x", err);
        return -6;
    }
    
    // Prepare the ADC to be able to sample
    err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_saadc_mode_trigger error: %08x", err);
        return -7;
    }

    // Configuring PPI
    uint8_t m_saadc_sample_ppi_channel;
    uint8_t m_saadc_start_ppi_channel;

    // PPI configure
    err = nrfx_gppi_channel_alloc(&m_saadc_sample_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate GPPI : %08x", err);
        return -8;
    }

    err = nrfx_gppi_channel_alloc(&m_saadc_start_ppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate a GPPI channels : %08x", err);
        return -9;
    }

    // Configure GPPI channels endpoints
    nrfx_gppi_channel_endpoints_setup(m_saadc_sample_ppi_channel,
    nrfx_timer_compare_event_address_get(Target_Timer, NRF_TIMER_CC_CHANNEL0),
    nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    nrfx_gppi_channel_endpoints_setup(m_saadc_start_ppi_channel,
    nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
    nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    nrfx_gppi_channels_enable(BIT(m_saadc_sample_ppi_channel));
    nrfx_gppi_channels_enable(BIT(m_saadc_start_ppi_channel));

    // Launch the timer
    nrfx_timer_enable(Target_Timer);
    return 0;
}

int SAADC_Stop(nrfx_timer_t *Target_Timer){
    nrfx_timer_disable(Target_Timer);
    return 0;
}

static void saadc_event_handler(nrfx_saadc_evt_t const * p_event)
{
    nrfx_err_t err;
    switch (p_event->type)
    {
        // Buffer has been initialized, we're ready. Then, we start the timer !
                      
        
        // The SAADC need a new buffer
        case NRFX_SAADC_EVT_BUF_REQ:
        
            /* STEP 5.2 - Set up the next available buffer. Alternate between buffer 0 and 1 */
            err = nrfx_saadc_buffer_set(saadc_buffer[(saadc_buffer_index++)%2], SAADC_BUFFER_SIZE);
            //err = nrfx_saadc_buffer_set(saadc_sample_buffer[((saadc_current_buffer == 0 )? saadc_current_buffer++ : 0)], SAADC_BUFFER_SIZE);
            if (err != NRFX_SUCCESS) {
                LOG_ERR("nrfx_saadc_buffer_set error: %08x", err);
                return;
            }
            break;

        case NRFX_SAADC_EVT_DONE:

            /* STEP 5.3 - Buffer has been filled. Do something with the data and proceed */
            int64_t average = 0;
            int16_t max = INT16_MIN;
            int16_t min = INT16_MAX;
            int16_t current_value; 

            int16_t channels_data[SAADC_INPUT_COUNT][SAADC_BUFFER_SIZE / SAADC_INPUT_COUNT] = {0};
            int16_t sample_number = 0;

            for(int i=0; i < p_event->data.done.size; i++){
                current_value = ((int16_t *)(p_event->data.done.p_buffer))[i];
                average += current_value;
                if(current_value > max){
                    max = current_value;
                }
                if(current_value < min){
                    min = current_value;
                }

                channels_data[i % SAADC_INPUT_COUNT][sample_number] = current_value;  
                if ((i % SAADC_INPUT_COUNT) == (SAADC_INPUT_COUNT - 1))
                    sample_number += 1;
            }
            average = average/p_event->data.done.size;
            LOG_INF("SAADC buffer at 0x%x filled with %d samples", (uint32_t)p_event->data.done.p_buffer, p_event->data.done.size);
            LOG_INF("AVG=%d, MIN=%d, MAX=%d", (int16_t)average, min, max);

            for (uint8_t k = 0; k < SAADC_INPUT_COUNT; k++)
                LOG_HEXDUMP_INF(channels_data[k], SAADC_BUFFER_SIZE / SAADC_INPUT_COUNT ,"Channel %d data", (k + 1));
            break;
        
        // Unknown event...
        default:
            LOG_WRN("Unhandled SAADC evt %d", p_event->type);
            break;
    }
}


