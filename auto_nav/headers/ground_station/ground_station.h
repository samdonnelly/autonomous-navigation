/**
 * @file ground_station.h
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

#ifndef _GROUND_STATION_H_ 
#define _GROUND_STATION_H_ 

//=======================================================================================
// Includes 

#include "includes_drivers.h" 
#include "nrf24l01_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define GS_MAX_CMD_LEN 32    // Max user input command length (bytes) 
#define GS_ADC_BUFF_SIZE 2   // Number of ADCs used 

//=======================================================================================


//=======================================================================================
// Classes 

class GroundStation
{
private:   // Private members 

    // User command data 
    uint8_t cb[GS_MAX_CMD_LEN];            // Circular buffer (CB) for user inputs 
    uint8_t cb_index;                      // CB index used for parsing commands 
    uint8_t cmd_buff[GS_MAX_CMD_LEN];      // User command parsed from the CB 
    uint8_t cmd_id[GS_MAX_CMD_LEN];        // ID from the user command 
    uint8_t cmd_value;                     // Value from the user command 
    uint8_t cmd_str[GS_MAX_CMD_LEN];       // String from the user command 

    // Timing 
    TIM_TypeDef *timer_nonblocking;        // Timer used for non-blocking delays 
    tim_compare_t delay_timer;             // Delay timing info 

    // System data 
    uint16_t adc_buff[GS_ADC_BUFF_SIZE];   // ADC buffer - thruster potentiometers 

    // Payload data 
    uint8_t read_buff[GS_MAX_CMD_LEN];     // Data read by PRX from PTX device 

public:   // Public member functions 

    // Constructor 
    GroundStation() 
        : cb_index(CLEAR), 
          cmd_value(CLEAR) 
    {
        memset((void *)cb, CLEAR, sizeof(cb)); 
        memset((void *)cmd_buff, CLEAR, sizeof(cmd_buff)); 
        memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
        memset((void *)cmd_str, CLEAR, sizeof(cmd_str)); 
        memset((void *)adc_buff, CLEAR, sizeof(adc_buff)); 
        memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
    } 

    // Destructor 
    ~GroundStation() {}

    // Setup 
    void GroundStationSetup(void); 

    // Application 
    void GroundStationApp(void); 
}; 

extern GroundStation ground_station; 

//=======================================================================================


//=======================================================================================
// Functions 

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

#endif   // _GROUND_STATION_H_ 
