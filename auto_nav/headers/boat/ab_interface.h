/**
 * @file ab_interface.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief 
 * 
 * @version 0.1
 * @date 2024-03-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _AB_INTERFACE_H_ 
#define _AB_INTERFACE_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Data sizes 
#define AB_ADC_BUFF_SIZE 3           // Size according to the number of ADCs used 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Autonomous boat initialization 
 * 
 * @details 
 */
void ab_init(void); 


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

#endif   // _AB_INTERFACE_H_ 
