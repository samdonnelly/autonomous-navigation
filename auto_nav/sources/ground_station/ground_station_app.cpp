/**
 * @file ground_station_app.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station application 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "ground_station.h" 
#include "stm32f4xx_it.h" 
#include "vehicle_radio_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Joysticks 
#define GS_ADC_REV_LIM 100           // ADC value reverse command limit 
#define GS_ADC_FWD_LIM 155           // ADC value forward command limit 
#define GS_JOYSTICK_NEUTRAL 0        // Joystick neutral (zero) position 

// Timing 
#define GS_ACTION_PERIOD 100000      // Time between throttle command sends (us) 
#define GS_HB_SEND_COUNTER 20        // This * RC_GS_ACTION_PERIOD gives HB send period 
#define GS_HB_TIMEOUT_COUNTER 100    // This * RC_GS_ACTION_PERIOD gives HB timeout 

//=======================================================================================


//=======================================================================================
// Enums 

// UI line index 
enum gs_ui_line_index : uint8_t 
{
    GS_UI_LINE_0, 
    GS_UI_LINE_1, 
    GS_UI_LINE_2, 
    GS_UI_LINE_3, 
    GS_UI_LINE_4, 
    GS_UI_LINE_5, 
    GS_UI_LINE_6, 
    GS_UI_LINE_7, 
    GS_UI_LINE_8, 
    GS_UI_LINE_9 
}; 

//=======================================================================================


//=======================================================================================
// Ground Station Application 

void GroundStation::GroundStationApp(void)
{
    //==================================================
    // Check for user serial terminal input 

    if (handler_flags.usart2_flag)
    {
        handler_flags.usart2_flag = CLEAR; 

        // Copy the new contents in the circular buffer to the user input buffer 
        dma_cb_index(dma_stream, &dma_index, &cb_index); 
        cb_parse(cb, &cb_index, cmd_buff); 

        // Display the input before performing any other actions 
        uart_cursor_move(uart, UART_CURSOR_UP, GS_UI_LINE_1); 
        LastUserInputUI(); 

        // Check the input against available commands for the ground station. If 
        // valid then execute a ground station callback function. If not recognized 
        // then send the input out via radio (i.e. to a vehicle). 
        if (!CommandLookUp(cmd_buff, command_table, ground_station))
        {
            // Input not recognized. Trigger a send via the RF module if the ground 
            // station isn't in manual control mode. 
            if (!gs_flags.manual_control_flag)
            {
                // Only update the write buffer if there is a non-empty input. This 
                // allows the user to send the same command again by simply hitting 
                // "enter" at the serial terminal. 
                if (*cmd_buff != NULL_CHAR)
                {
                    memcpy((void *)write_buff, (void *)cmd_buff, GS_MAX_CMD_LEN); 
                }
                
                gs_flags.user_cmd_flag = SET_BIT; 
            }
        }

        // Clear the user command prompt 
        CmdPromptUI(); 
    }
    
    //==================================================

    //==================================================
    // Check for periodic action items 

    if (tim_compare(timer_nonblocking, 
                    delay_timer.clk_freq, 
                    GS_ACTION_PERIOD, 
                    &delay_timer.time_cnt_total, 
                    &delay_timer.time_cnt, 
                    &delay_timer.time_start))
    {
        // time_start flag does not need to be set again because this timer runs 
        // continuously. 
        // This timer should be the lowest common denominator of time for each 
        // operation. 

        //==================================================
        // Send commands to the vehicle 

        // Check for which message/command to send to a vehicle 
        if (gs_flags.manual_control_flag)
        {
            ManualControlMode(); 
        }
        else if (gs_flags.user_cmd_flag)
        {
            SendUserCmd(); 
        }
        else 
        {
            SendHeartbeat(); 
        }
        
        //==================================================


        //==================================================
        // Look for responses from the vehicles 

        // Look for incoming messages 
        MsgCheck(); 

        // Update the radio connection status 
        RadioConnectionStatus(); 

        //==================================================
    }
    
    //==================================================
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Manual control mode 
void GroundStation::ManualControlMode(void)
{
    static gpio_pin_state_t led_state = GPIO_LOW; 
    static uint8_t joystick = CLEAR_BIT; 

    int16_t joystick_position = GS_JOYSTICK_NEUTRAL; 
    uint16_t adc_value = adc_buff[joystick]; 
    char side = (joystick) ? GS_RADIO_LEFT_JOYSTICK : GS_RADIO_RIGHT_JOYSTICK; 
    char sign = GS_RADIO_NEUTRAL; 

    //==================================================
    // Map the ADC value to a joystick input 

    // Check the position of the joystick 
    if (adc_value > GS_ADC_FWD_LIM)
    {
        joystick_position = (int16_t)adc_value - GS_ADC_FWD_LIM; 
        sign = GS_RADIO_FWD_DIRECTION; 
    }
    else if (adc_value < GS_ADC_REV_LIM)
    {
        joystick_position = (int16_t)adc_value - GS_ADC_REV_LIM; 
        joystick_position = ~joystick_position + 1; 
        sign = GS_RADIO_REV_DIRECTION; 
    }
    
    //==================================================

    //==================================================
    // Format and send the joystick command to the vehicle 

    // Format the payload with the joystick ID, sign and value then send the payload. 
    snprintf(
        (char *)write_buff, 
        GS_MAX_CMD_LEN, 
        "%c%c %d", 
        side, sign, joystick_position); 

    if (nrf24l01_send_payload(write_buff) == NRF24L01_OK)
    {
        led_state = (gpio_pin_state_t)(GPIO_HIGH - led_state); 
        gpio_write(GPIOA, GPIOX_PIN_5, led_state); 
    }

    //==================================================

    // Toggle the joystick flag (alternate between joysticks) 
    joystick = SET_BIT - joystick; 
}


// Send user input 
void GroundStation::SendUserCmd(void)
{
    gs_flags.user_cmd_flag = CLEAR_BIT; 

    // Clear the most recent message received interface 
    memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
    VehicleMessageUI(); 

    // Send the user input 
    status_message = (nrf24l01_send_payload(write_buff) == NRF24L01_OK) ? 
                     gs_ui_status_cmd_sent : gs_ui_status_fail; 
    CmdStatusUI(); 
    CmdPromptUI(); 
}


// Send heartbeat 
void GroundStation::SendHeartbeat(void)
{
    static uint8_t hb_send_counter = CLEAR; 

    // Send a heartbeat message to the remote system periodically 
    if (hb_send_counter++ >= GS_HB_SEND_COUNTER)
    {
        hb_send_counter = CLEAR; 
        nrf24l01_send_payload((uint8_t *)vehicle_radio_cmd_ping); 
    }
}


// Check for an incoming message 
void GroundStation::MsgCheck(void)
{
    // Look for an incoming message from the remote system 
    if (nrf24l01_data_ready_status() == nrf24l01_pipe)
    {
        nrf24l01_receive_payload(read_buff); 

        // Clear the timeout for any message received 
        hb_timeout_counter = CLEAR; 
        gs_flags.radio_connection_flag = SET_BIT; 

        // If the message received is not a ping/heartbeat response then display the 
        // response for the user to see. Heartbeat responses are only used to reset the 
        // heartbeat timeout. 
        if (strcmp((char *)read_buff, vehicle_radio_ping_confirm) != 0)
        {
            VehicleMessageUI(); 
            CmdPromptUI(); 
        }
    }
}


// Check the radio connection 
void GroundStation::RadioConnectionStatus(void)
{
    // Check if the radio connection has been lost for too long 
    if (hb_timeout_counter++ >= GS_HB_TIMEOUT_COUNTER)
    {
        hb_timeout_counter = CLEAR; 
        gs_flags.radio_connection_flag = CLEAR_BIT; 
    }
}

//=======================================================================================


//=======================================================================================
// Command callbacks 

// Manual control - on/off toggle option 
void GroundStation::ManualControlCmd(
    GroundStation& gs_radio, 
    uint8_t *manual_cmd_arg)
{
    if (manual_cmd_arg == nullptr)
    {
        return; 
    }

    // Check for an "on" or "off" request 
    if (strcmp((char *)manual_cmd_arg, gs_sub_cmd_on) == 0)
    {
        gs_radio.gs_flags.manual_control_flag = SET_BIT; 
        gs_radio.status_message = gs_ui_status_success; 
        gpio_write(GPIOA, GPIOX_PIN_5, GPIO_HIGH);   // Turn on board LED 

        // Disable other commands 
        for (uint8_t i = 1; i < GS_NUM_CMDS; i++)
        {
            gs_radio.CommandEnable(gs_radio.command_table[i].cmd, 
                                   gs_radio.command_table, CLEAR_BIT); 
        }
    }
    else if (strcmp((char *)manual_cmd_arg, gs_sub_cmd_off) == 0)
    {
        gs_radio.gs_flags.manual_control_flag = CLEAR_BIT; 
        gs_radio.status_message = gs_ui_status_success; 
        gpio_write(GPIOA, GPIOX_PIN_5, GPIO_LOW);   // Turn board LED off 

        // Enable other commands 
        for (uint8_t i = 1; i < GS_NUM_CMDS; i++)
        {
            gs_radio.CommandEnable(gs_radio.command_table[i].cmd, 
                                   gs_radio.command_table, SET_BIT); 
        }
    }
    else 
    {
        gs_radio.status_message = gs_ui_status_invalid; 
    }

    gs_radio.CmdStatusUI(); 
}


// Update radio connection status in the user interface 
void GroundStation::UpdateRadioStatus(
    GroundStation& gs_radio, 
    uint8_t *update_cmd_arg)
{
    gs_radio.status_message = gs_ui_status_success; 
    gs_radio.CmdStatusUI(); 
    gs_radio.RadioConnectionUI(); 
}


// RF frequency channel set 
void GroundStation::RFChannelSetCmd(
    GroundStation& gs_radio, 
    uint8_t *rf_channel_cmd_arg)
{
    if (*rf_channel_cmd_arg <= NRF24L01_RF_CH_MAX)
    {
        nrf24l01_set_rf_ch(*rf_channel_cmd_arg); 

        if (nrf24l01_rf_ch_write() == NRF24L01_OK)
        {
            gs_radio.RFChannelUI(); 
            gs_radio.status_message = gs_ui_status_success; 
        }
        else 
        {
            gs_radio.status_message = gs_ui_status_fail; 
        }
    }
    else 
    {
        gs_radio.status_message = gs_ui_status_invalid; 
    }

    gs_radio.CmdStatusUI(); 
}


// RF power output set 
void GroundStation::RFPwrOutputSetCmd(
    GroundStation& gs_radio, 
    uint8_t *rf_power_cmd_arg)
{
    if (*rf_power_cmd_arg <= (uint8_t)NRF24L01_RF_PWR_0DBM)
    {
        nrf24l01_set_rf_setup_pwr((nrf24l01_rf_pwr_t)(*rf_power_cmd_arg)); 

        if (nrf24l01_rf_setup_write() == NRF24L01_OK)
        {
            gs_radio.RFPwrOutputUI(); 
            gs_radio.status_message = gs_ui_status_success; 
        }
        else 
        {
            gs_radio.status_message = gs_ui_status_fail; 
        }
    }
    else 
    {
        gs_radio.status_message = gs_ui_status_invalid; 
    }

    gs_radio.CmdStatusUI(); 
}


// RF data rate set 
void GroundStation::RFDataRateSetCmd(
    GroundStation& gs_radio, 
    uint8_t *rf_dr_cmd_arg)
{
    if (*rf_dr_cmd_arg <= (uint8_t)NRF24L01_DR_250KBPS)
    {
        nrf24l01_set_rf_setup_dr((nrf24l01_data_rate_t)(*rf_dr_cmd_arg)); 

        if (nrf24l01_rf_setup_write() == NRF24L01_OK)
        {
            gs_radio.RFDataRateUI(); 
            gs_radio.status_message = gs_ui_status_success; 
        }
        else 
        {
            gs_radio.status_message = gs_ui_status_fail; 
        }
    }
    else 
    {
        gs_radio.status_message = gs_ui_status_invalid; 
    }

    gs_radio.CmdStatusUI(); 
}


// RF data pipe set 
void GroundStation::RFDatePipeSetCmd(
    GroundStation& gs_radio, 
    uint8_t *rf_dp_cmd_arg)
{
    // Switch between data pipes being monitored 
}

//=======================================================================================


//=======================================================================================
// User interface 

// UI init 
void GroundStation::InitializeUI(void)
{
    uart_cursor_move(uart, UART_CURSOR_DOWN, GS_UI_LINE_9); 
    status_message = " "; 

    LastUserInputUI(); 
    RadioConnectionUI(); 
    VehicleMessageUI(); 
    CmdStatusUI(); 
    RFChannelUI(); 
    RFDataRateUI(); 
    RFPwrOutputUI(); 
    RFDataPipeUI(); 
    CmdPromptUI(); 
}


// Last user input 
void GroundStation::LastUserInputUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_last_input, (char *)cmd_buff); 
    WriteLineUI(GS_UI_LINE_9); 
}


// Radio connection status 
void GroundStation::RadioConnectionUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_radio_connect, gs_flags.radio_connection_flag); 
    WriteLineUI(GS_UI_LINE_8); 
}


// Vehicle message 
void GroundStation::VehicleMessageUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_vehicle_msg, (char *)read_buff); 
    WriteLineUI(GS_UI_LINE_7); 
}


// Command Feedback 
void GroundStation::CmdStatusUI(void)
{
    if (status_message != nullptr)
    {
        snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_cmd_status, status_message); 
        WriteLineUI(GS_UI_LINE_6); 
    }
}


// RF module frequency channel UI 
void GroundStation::RFChannelUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_channel_set, nrf24l01_get_rf_ch()); 
    WriteLineUI(GS_UI_LINE_5); 
}


// RF module data rate UI 
void GroundStation::RFDataRateUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_dr_set, (uint8_t)nrf24l01_get_rf_setup_dr()); 
    WriteLineUI(GS_UI_LINE_4); 
}


// RF module power output UI 
void GroundStation::RFPwrOutputUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_pwr_set, (uint8_t)nrf24l01_get_rf_setup_pwr()); 
    WriteLineUI(GS_UI_LINE_3); 
}


// RF module data pipe UI 
void GroundStation::RFDataPipeUI(void)
{
    snprintf(ui_buff, GS_UI_BUFF_SIZE, gs_ui_dp_set, (uint8_t)NRF24L01_DP_1); 
    WriteLineUI(GS_UI_LINE_2); 
}


// Command prompt 
void GroundStation::CmdPromptUI(void)
{
    uart_send_str(uart, gs_ui_cmd_prompt); 
}


// Write a line of data 
void GroundStation::WriteLineUI(uint8_t line_offset)
{
    uart_cursor_move(uart, UART_CURSOR_UP, line_offset); 
    uart_send_str(uart, "\r"); 
    uart_send_str(uart, ui_buff); 
    uart_send_str(uart, "\033[K");   // Clear the line to the right 
    uart_cursor_move(uart, UART_CURSOR_DOWN, line_offset); 
}

//=======================================================================================
