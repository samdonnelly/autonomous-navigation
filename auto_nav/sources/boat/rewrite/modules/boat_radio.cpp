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

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// User functions 

// Command read 
void BoatRadio::CommandRead(Boat& boat_radio)
{
    // Heartbeat check 
    // Increment the timeout counter periodically until the timeout limit at which 
    // point the system assumes to have lost radio connection. Connection status is 
    // re-established once a HB command is seen. 

    // Check if a payload has been received 
    if (nrf24l01_data_ready_status(pipe))
    {
        // Payload has been received. Read the payload from the device RX FIFO and 
        // queue a data check event. 
        memset((void *)read_buff, CLEAR, sizeof(read_buff)); 
        nrf24l01_receive_payload(read_buff, pipe); 
        boat_radio.MainEventQueue((Event)Boat::MainEvents::RADIO_CHECK); 
    }
}


// Command check 
void BoatRadio::CommandCheck(Boat& boat_radio)
{
    // byte 0 is the message length so the message starts at byte 1 
    if (CommandLookUp(&read_buff[1], command_table, boat_radio))
    {
        // Reset connection status timeout (heartbeat check) 
    }
}

//=======================================================================================


//=======================================================================================
// Command callbacks 

// Heartbeat command 
void BoatRadio::HBCmd(
    Boat& boat_radio, 
    uint8_t hb_cmd_value)
{
    // This function is here to provide a match for a valid command check. It's meant to 
    // verify that there is a radio connection but the connection flag gets set when any 
    // command matches. 
}


// Idle (standby) state command 
void BoatRadio::IdleCmd(
    Boat& boat_radio, 
    uint8_t idle_cmd_value)
{
    boat_radio.main_flags.standby_state = SET_BIT; 
    boat_radio.MainStateChange(); 
}


// Autonomous state command 
void BoatRadio::AutoCmd(
    Boat& boat_radio, 
    uint8_t auto_cmd_value)
{
    boat_radio.main_flags.auto_state = SET_BIT; 
    boat_radio.MainStateChange(); 
}


// Manual control state command 
void BoatRadio::ManualCmd(
    Boat& boat_radio, 
    uint8_t manual_cmd_value)
{
    boat_radio.main_flags.manual_state = SET_BIT; 
    boat_radio.MainStateChange(); 
}


// Waypoint mission index update command 
void BoatRadio::IndexCmd(
    Boat& boat_radio, 
    uint8_t index_cmd_value)
{
    // static uint8_t index_check = CLEAR; 
    // static uint8_t index_last = CLEAR; 

    // // Compare the previous index command to the new index command. The radio messages 
    // // between the ground station and boat are poor meaning a complete and correct 
    // // message often does not get transmitted and received successfully. This can lead 
    // // to the index not being updated to the desired value and therefore the boat moving 
    // // to a target it's not supposed to. To combat this, the index has to been seen 
    // // successively at least "AB_GPS_INDEX_CNT" times before the index will be updated. 
    // if (index_cmd_value != index_last)
    // {
    //     index_last = index_cmd_value; 
    //     index_check = SET_BIT; 
    // }
    // else 
    // {
    //     index_check++; 
    // }

    // // Check that the index is within bounds and seen the last AB_GPS_INDEX_CNT times before 
    // // updating the index (filters noise). 
    // if ((index_cmd_value < NUM_GPS_WAYPOINTS) && (index_check >= AB_GPS_INDEX_CNT))
    // {
    //     boat_test.waypoint_index = index_cmd_value; 
    //     boat_test.target.lat = gps_waypoints[boat_test.waypoint_index].lat; 
    //     boat_test.target.lon = gps_waypoints[boat_test.waypoint_index].lon; 
    //     index_check = CLEAR; 
    // }
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
    CommandEnable(boat_radio_ping, command_table, cmd_state); 
    CommandEnable(boat_radio_auto, command_table, cmd_state); 
    CommandEnable(boat_radio_manual, command_table, cmd_state); 
    CommandEnable(boat_radio_index, command_table, cmd_state); 
}


// Main thread: Auto state 
void BoatRadio::MainAutoStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(boat_radio_ping, command_table, cmd_state); 
    CommandEnable(boat_radio_idle, command_table, cmd_state); 
    CommandEnable(boat_radio_index, command_table, cmd_state); 
}


// Main thread: Manual state 
void BoatRadio::MainManualStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(boat_radio_ping, command_table, cmd_state); 
    CommandEnable(boat_radio_idle, command_table, cmd_state); 
    CommandEnable(boat_radio_RP, command_table, cmd_state); 
    CommandEnable(boat_radio_RN, command_table, cmd_state); 
    CommandEnable(boat_radio_LP, command_table, cmd_state); 
    CommandEnable(boat_radio_LN, command_table, cmd_state); 
}


// Main thread: Low Power state 
void BoatRadio::MainLowPwrStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(boat_radio_ping, command_table, cmd_state); 
}


// Main thread: Fault state 
void BoatRadio::MainFaultStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(boat_radio_ping, command_table, cmd_state); 
}

//=======================================================================================
