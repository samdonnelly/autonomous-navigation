/**
 * @file project_init.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat initialization code 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "ab_app.h"

//=======================================================================================


//=======================================================================================
// Global variables 

// Data record instance 
static ab_data_t ab_data; 

//=======================================================================================


//=======================================================================================
// Functions 

// Autonomous boat application  
void ab_app_init(
    DMA_Stream_TypeDef *adc_dma_stream, 
    ADC_TypeDef *adc)
{
    // Autonomous boat application code initialization 

    // Configure the DMA stream 
    dma_stream_config(
        adc_dma_stream, 
        (uint32_t)(&adc->DR), 
        (uint32_t)ab_data.adc_buff, 
        (uint16_t)AB_ADC_BUFF_SIZE); 
}


// Autonomous boat application  
void ab_app(void)
{
    // Autonomous boat application code 
}

//=======================================================================================
