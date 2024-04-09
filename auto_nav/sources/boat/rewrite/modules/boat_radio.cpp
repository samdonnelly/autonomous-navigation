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

//=======================================================================================


//=======================================================================================
// User functions 

void BoatRadio::CmdRead(void)
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
}

//=======================================================================================


//=======================================================================================
// Boat commands 

// Idle command 
void BoatRadio::IdleCmd(
    Boat& boat_radio, 
    uint8_t idle_cmd_value)
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


// Autonomous mode command 
void BoatRadio::AutoCmd(
    Boat& boat_radio, 
    uint8_t auto_cmd_value)
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


// Heartbeat command 
void BoatRadio::HBCmd(
    Boat& boat_radio, 
    uint8_t hb_cmd_value)
{
    // 
}

//=======================================================================================
