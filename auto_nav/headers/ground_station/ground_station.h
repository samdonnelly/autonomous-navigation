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
#include "radio_module.h" 
#include "nrf24l01_config.h" 
#include "commands_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define GS_MAX_CMD_LEN 32    // Max user input command length (bytes) 
#define GS_ADC_BUFF_SIZE 2   // Number of ADCs used 

//=======================================================================================


//=======================================================================================
// Classes 

class GroundStation : public RadioModule<GroundStation, GS_NUM_CMDS> 
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
    uint8_t hb_timeout_counter;            // Heartbeat response timeout counter 

    // System data 
    uint16_t adc_buff[GS_ADC_BUFF_SIZE];   // ADC buffer - thruster potentiometers 

    // Payload data 
    uint8_t read_buff[GS_MAX_CMD_LEN];     // Data read by PRX from PTX device 
    // uint8_t msg_buff[GS_MAX_CMD_LEN];      // Vehicle messages that aren't heartbeat responses 
    uint8_t write_buff[GS_MAX_CMD_LEN];    // Data sent by PTX to a PRX device 

    // Flags 
    struct GSFlags 
    {
        // Status flags 
        uint8_t user_cmd_flag         : 1; 
        uint8_t manual_control_flag   : 1; 
        uint8_t radio_connection_flag : 1; 
    }
    gs_flags; 

public:   // Public member functions 

    // Constructor 
    GroundStation() 
        : cb_index(CLEAR), 
          cmd_value(CLEAR), 
          hb_timeout_counter(CLEAR) 
    {
        memset((void *)cb, CLEAR, sizeof(cb)); 
        memset((void *)cmd_buff, CLEAR, sizeof(cmd_buff)); 
        memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
        memset((void *)cmd_str, CLEAR, sizeof(cmd_str)); 
        memset((void *)adc_buff, CLEAR, sizeof(adc_buff)); 
        memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
        memset((void *)write_buff, CLEAR, sizeof(write_buff)); 

        gs_flags.user_cmd_flag = CLEAR_BIT; 
        gs_flags.manual_control_flag = CLEAR_BIT; 
        gs_flags.radio_connection_flag = CLEAR_BIT; 

        // Timing info is configured in the setup function 
    } 

    // Destructor 
    ~GroundStation() {}

    // Setup 
    void GroundStationSetup(void); 

    // Application 
    void GroundStationApp(void); 

private:   // Private member functions 

    //==================================================
    // Helper functions 

    // Manual control mode 
    void ManualControlMode(void); 

    // ADC throttle mapping 
    int16_t ADCThrottleMapping(uint16_t adc_value); 

    // Send user input 
    void SendUserCmd(void); 

    // Send heartbeat 
    void SendHeartbeat(void); 

    // Check for an incoming message 
    void MsgCheck(void); 

    // Check the radio connection 
    void RadioConnectionStatus(void); 
    
    //==================================================

    //==================================================
    // Commands 

    // Command callbacks 
    static void ManualControlCmd(GroundStation& gs_radio, uint8_t *manual_cmd_arg); 
    static void RFChannelSetCmd(GroundStation& gs_radio, uint8_t *rf_channel_cmd_arg); 
    static void RFPwrOutputSetCmd(GroundStation& gs_radio, uint8_t *rf_power_cmd_arg); 
    static void RFDataRateSetCmd(GroundStation& gs_radio, uint8_t *rf_dr_cmd_arg); 
    static void RFDatePipeSetCmd(GroundStation& gs_radio, uint8_t *rf_dp_cmd_arg); 
    static void UpdateOutputData(GroundStation& gs_radio, uint8_t *update_cmd_arg); 

    // Command table 
    std::array<RadioCmdData, GS_NUM_CMDS> command_table = 
    {{
        // User commands 
        {gs_cmd_manual,     CMD_ARG_STR,   &ManualControlCmd,  CLEAR_BIT}, 
        {gs_cmd_rf_channel, CMD_ARG_VALUE, &RFChannelSetCmd,   CLEAR_BIT}, 
        {gs_cmd_rf_power,   CMD_ARG_VALUE, &RFPwrOutputSetCmd, CLEAR_BIT}, 
        {gs_cmd_rf_dr,      CMD_ARG_VALUE, &RFDataRateSetCmd,  CLEAR_BIT}, 
        {gs_cmd_rf_dp,      CMD_ARG_VALUE, &RFDatePipeSetCmd,  CLEAR_BIT}, 
        {gs_cmd_update,     CMD_ARG_NONE,  &UpdateOutputData,  CLEAR_BIT} 
    }}; 
    
    //==================================================
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
