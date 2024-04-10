/**
 * @file boat_radio.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat radio 
 * 
 * @version 0.1
 * @date 2024-04-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat_radio.h" 
#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Messages 

const std::string 
boat_radio_ping = "ping",       // 0. Ping (heartbeat) 
boat_radio_idle = "idle",       // 1. Idle (standby) state 
boat_radio_auto = "auto",       // 2. Autonomous state 
boat_radio_manual = "manual",   // 3. Manual (remote) control state 
boat_radio_index = "index",     // 4. Waypoint index set 
boat_radio_RP = "RP",           // 5. Right thruster - forward thrust 
boat_radio_RN = "RN",           // 6. Right thruster - reverse thrust 
boat_radio_LP = "LP",           // 7. Left thruster - forward thrust 
boat_radio_LN = "LN";           // 8. Left thruster - reverse thrust 

//=======================================================================================


//=======================================================================================
// User functions 

// Command read 
void BoatRadio::CommandRead(Boat& boat_radio)
{
    // // Heartbeat check 
    // // Increment the timeout counter periodically until the timeout limit at which 
    // // point the system assumes to have lost radio connection. Connection status is 
    // // re-established once a HB command is seen. 
    // if (tim_compare(timer_nonblocking, 
    //                 hb_timer.clk_freq, 
    //                 AB_HB_PERIOD, 
    //                 &hb_timer.time_cnt_total, 
    //                 &hb_timer.time_cnt, 
    //                 &hb_timer.time_start))
    // {
    //     if (hb_timeout >= AB_HB_TIMEOUT)
    //     {
    //         connect_flag = CLEAR_BIT; 
    //     }
    //     else 
    //     {
    //         hb_timeout++; 
    //     }
    // }

    // // External command check 
    // // Check if a payload has been received 
    // if (nrf24l01_data_ready_status(pipe))
    // {
    //     // Payload has been received. Read the payload from the device RX FIFO. 
    //     nrf24l01_receive_payload(read_buff, pipe); 

    //     memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
    // }

    // const auto& cmd = command_table; 
    // if (cmd.find("auto") != cmd.end())
    // {
    //     // 
    // }

    // If new data is available then queue a data check 
    boat_radio.MainEventQueue((Event)Boat::MainEvents::RADIO_CHECK); 
}


// Command check 
void BoatRadio::CommandCheck(Boat& boat_radio)
{
    // First byte (0th byte) is the message length 
    CommandLookUp(&read_buff[1], command_table, boat_radio); 
}

//=======================================================================================


//=======================================================================================
// Command callbacks 

// Heartbeat command 
void BoatRadio::HBCmd(
    Boat& boat_radio, 
    uint8_t hb_cmd_value)
{
    // 
}


// Idle command 
void BoatRadio::IdleCmd(
    Boat& boat_radio, 
    uint8_t idle_cmd_value)
{
    // 
}


// Autonomous mode command 
void BoatRadio::AutoCmd(
    Boat& boat_radio, 
    uint8_t auto_cmd_value)
{
    // 
}


// Manual control mode command 
void BoatRadio::ManualCmd(
    Boat& boat_radio, 
    uint8_t manual_cmd_value)
{
    // 
}


// Index update command 
void BoatRadio::IndexCmd(
    Boat& boat_radio, 
    uint8_t index_cmd_value)
{
    // 
}


// Manual throttle command 
void BoatRadio::ThrottleCmd(
    Boat& boat_radio, 
    uint8_t throttle_cmd_value)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Command enable/disable 

// Main thread: Standby state 
void BoatRadio::MainStandbyStateCmdEnable(uint8_t cmd_state)
{
    command_table[boat_radio_auto].cmd_enable = cmd_state; 
    command_table[boat_radio_manual].cmd_enable = cmd_state; 
    command_table[boat_radio_index].cmd_enable = cmd_state; 
    command_table[boat_radio_ping].cmd_enable = cmd_state; 
}


// Main thread: Auto state 
void BoatRadio::MainAutoStateCmdEnable(uint8_t cmd_state)
{
    command_table[boat_radio_idle].cmd_enable = cmd_state; 
    command_table[boat_radio_RP].cmd_enable = cmd_state; 
    command_table[boat_radio_RN].cmd_enable = cmd_state; 
    command_table[boat_radio_LP].cmd_enable = cmd_state; 
    command_table[boat_radio_LN].cmd_enable = cmd_state; 
    command_table[boat_radio_ping].cmd_enable = cmd_state; 
}


// Main thread: Manual state 
void BoatRadio::MainManualStateCmdEnable(uint8_t cmd_state)
{
    command_table[boat_radio_idle].cmd_enable = cmd_state; 
    command_table[boat_radio_index].cmd_enable = cmd_state; 
    command_table[boat_radio_ping].cmd_enable = cmd_state; 
}


// Main thread: Low Power state 
void BoatRadio::MainLowPwrStateCmdEnable(uint8_t cmd_state)
{
    command_table[boat_radio_ping].cmd_enable = cmd_state; 
}


// Main thread: Fault state 
void BoatRadio::MainFaultStateCmdEnable(uint8_t cmd_state)
{
    command_table[boat_radio_ping].cmd_enable = cmd_state; 
}

//=======================================================================================
