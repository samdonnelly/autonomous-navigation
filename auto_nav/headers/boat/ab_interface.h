/**
 * @file ab_interface.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat interface 
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

#define AB_ADC_BUFF_SIZE 3   // Number of ADCs used 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Autonomous boat initialization 
 * 
 * @details Initializes devices and peripherals used by the boat. Meant to be called 
 *          once at the start of the program. 
 */
void ab_init(void); 


/**
 * @brief Autonomous boat application initialization 
 * 
 * @details Initializes the data used to track and control the boat. Must be called once 
 *          at the start of the program. 
 * 
 * @param timer_nonblocking : timer port used for non-blocking delays 
 * @param adc_dma_stream : DMA stream for handling ADC readings 
 * @param adc : ADC port for ADC readings 
 * @param pipe_num : nRF24L01 data pipe number for talking to the ground station 
 */
void ab_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    nrf24l01_data_pipe_t pipe_num); 


/**
 * @brief Autonomous boat application 
 * 
 * @details Main control loop of the program that's called forever. 
 */
void ab_app(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_INTERFACE_H_ 
