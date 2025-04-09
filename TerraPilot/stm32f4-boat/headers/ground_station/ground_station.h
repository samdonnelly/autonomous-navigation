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
#include "gs_interface_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define GS_MAX_CMD_LEN 32    // Max user input command length (bytes) 
#define GS_ADC_BUFF_SIZE 2   // Number of ADCs used 
#define GS_UI_BUFF_SIZE 30   // Max user interface string output size 

//=======================================================================================


//=======================================================================================
// Classes 

class GroundStation : public RadioModule<GroundStation, GS_NUM_CMDS> 
{
private:   // Private members 

    // User interface 
    USART_TypeDef *uart;                     // UART for serial terminal 
    char ui_buff[GS_UI_BUFF_SIZE];           // Buffer to store RF settings string 
    const char *status_message;              // Pointer to status message for the user 

    // User command data 
    DMA_Stream_TypeDef *dma_stream; 
    uint8_t cb[GS_MAX_CMD_LEN];              // Circular buffer (CB) for user inputs 
    cb_index_t cb_index;                     // Circular buffer indexing info 
    dma_index_t dma_index;                   // DMA transfer indexing info 
    uint8_t cmd_buff[GS_MAX_CMD_LEN];        // User command parsed from the CB 
    uint8_t cmd_id[GS_MAX_CMD_LEN];          // ID from the user command 
    uint8_t cmd_value;                       // Value from the user command 
    uint8_t cmd_str[GS_MAX_CMD_LEN];         // String from the user command 

    // Timing 
    TIM_TypeDef *timer_nonblocking;          // Timer used for non-blocking delays 
    tim_compare_t delay_timer;               // Delay timing info 
    uint8_t hb_timeout_counter;              // Heartbeat response timeout counter 

    // System data 
    uint16_t adc_buff[GS_ADC_BUFF_SIZE];     // ADC buffer - thruster potentiometers 

    // Payload data 
    uint8_t read_buff[GS_MAX_CMD_LEN];       // Data read by PRX from PTX device 
    uint8_t write_buff[GS_MAX_CMD_LEN];      // Data sent by PTX to a PRX device 

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
    GroundStation(); 

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
    static void UpdateRadioStatus(GroundStation& gs_radio, uint8_t *update_cmd_arg); 
    static void RFChannelSetCmd(GroundStation& gs_radio, uint8_t *rf_channel_cmd_arg); 
    static void RFPwrOutputSetCmd(GroundStation& gs_radio, uint8_t *rf_power_cmd_arg); 
    static void RFDataRateSetCmd(GroundStation& gs_radio, uint8_t *rf_dr_cmd_arg); 
    static void RFDatePipeSetCmd(GroundStation& gs_radio, uint8_t *rf_dp_cmd_arg); 

    // Command table 
    std::array<RadioCmdData, GS_NUM_CMDS> command_table = 
    {{
        // User commands 
        {gs_cmd_manual,     CMD_ARG_STR,   &ManualControlCmd,  CLEAR_BIT}, 
        {gs_cmd_update,     CMD_ARG_NONE,  &UpdateRadioStatus, CLEAR_BIT}, 
        {gs_cmd_rf_channel, CMD_ARG_VALUE, &RFChannelSetCmd,   CLEAR_BIT}, 
        {gs_cmd_rf_power,   CMD_ARG_VALUE, &RFPwrOutputSetCmd, CLEAR_BIT}, 
        {gs_cmd_rf_dr,      CMD_ARG_VALUE, &RFDataRateSetCmd,  CLEAR_BIT}, 
        {gs_cmd_rf_dp,      CMD_ARG_VALUE, &RFDatePipeSetCmd,  CLEAR_BIT} 
    }}; 
    
    //==================================================

    //==================================================
    // User interface 

    // UI initialization 
    void InitializeUI(void); 

    // UI outputs 
    void LastUserInputUI(void);     // Last user input 
    void RadioConnectionUI(void);   // Radio connection status 
    void VehicleMessageUI(void);    // Vehicle message 
    void CmdStatusUI(void);         // Command Feedback 
    void RFChannelUI(void);         // RF frequency channel 
    void RFDataRateUI(void);        // RF data rate setting 
    void RFPwrOutputUI(void);       // RF power output setting 
    void RFDataPipeUI(void);        // RF data pipe 
    void CmdPromptUI(void);         // Command prompt 

    // Write a line of data 
    void WriteLineUI(uint8_t line_offset); 

    //==================================================
}; 

extern GroundStation ground_station; 

//=======================================================================================

#endif   // _GROUND_STATION_H_ 
