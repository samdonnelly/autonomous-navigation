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
// Macros 

// Data sizes 
#define AB_ADC_BUFF_SIZE 3           // Size according to the number of ADCs used 

//=======================================================================================


//=======================================================================================
// Structures 

// Data record for the system 
typedef struct ab_data_s 
{
    // Peripherals 
    ADC_TypeDef *adc;                        // ADC port battery soc and pots 

    // System data 
    uint16_t adc_buff[AB_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 
}
ab_data_t; 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief 
 * 
 * @details 
 * 
 * @param adc_dma_stream 
 * @param adc 
 */
void ab_app_init(
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc); 


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
