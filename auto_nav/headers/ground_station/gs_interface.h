/**
 * @file gs_interface.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station interface 
 * 
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _GS_INTERFACE_H_ 
#define _GS_INTERFACE_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Ground station initialization 
 * 
 * @details Called once on startup. 
 */
void gs_init(void); 


/**
 * @brief Ground station application initializzation 
 * 
 * @details Called once on startup to initialize the data record and DMA. 
 * 
 * @param timer_nonblocking : timer port used for non-blocing delays 
 * @param adc_dma_stream : DMA stream used for ADC reads 
 * @param adc : ADC port to read from 
 * @param uart_dma_stream : DMA stream used for UART reads 
 * @param uart : UART port to read from 
 */
void gs_app_init(
    TIM_TypeDef *timer_nonblocking, 
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc, 
    DMA_Stream_TypeDef *uart_dma_stream, 
    USART_TypeDef *uart); 


/**
 * @brief Ground station application 
 * 
 * @details Called repeatedly to execute the main functions of the ground station. 
 */
void gs_app(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _GS_INTERFACE_H_ 
