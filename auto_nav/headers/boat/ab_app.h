/**
 * @file project_init.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat application header 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _AB_APP_H_ 
#define _AB_APP_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "ab_includes_app.h" 
#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief 
 * 
 * @details 
 * 
 * @param adc_dma_stream : 
 * @param adc : 
 * @param timer_nonblocking : 
 * @param pipe_num : 
 */
void ab_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    nrf24l01_data_pipe_t pipe_num); 


/**
 * @brief Autonomous boat application 
 * 
 * @details 
 */
void ab_app(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_APP_H_ 
