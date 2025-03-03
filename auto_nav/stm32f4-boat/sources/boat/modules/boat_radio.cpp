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
// Macros 

#define GS_CONNECT_TIMEOUT 100   // Ground station radio connection timeout counter 

//=======================================================================================


//=======================================================================================
// User functions 

// Command read 
void BoatRadio::CommandRead(Boat& boat_radio)
{
    // 'gs_connect_timeout', 'gs_connect_flag', 'data_pipe' and 'read_buff' are not 
    // protected because 'CommandRead' and 'CommandCheck' are only called sequentially 
    // every 100ms so there is no threat of one interrupting the other. 

    //==================================================
    // Increment timeouts 

    // Ground station timeout 
    if (++gs_connect_timeout >= GS_CONNECT_TIMEOUT)
    {
        // A command from the ground station has not been received within the timeout 
        // limit so clear the connection status to indicate a loss of radio connection. 
        // Connection status will change when a command is received. 
        gs_connect_flag = CLEAR_BIT; 
        gs_connect_timeout = CLEAR; 
    }
    
    //==================================================

    //==================================================
    // Check for and read new data 

    data_pipe = nrf24l01_data_ready_status(); 

    // Check if a payload has been received 
    if (data_pipe != NRF24L01_RX_FIFO_EMPTY)
    {
        // New data available. Read the payload and queue a data check event. 
        nrf24l01_receive_payload(read_buff); 
        boat_radio.MainEventQueue((Event)Boat::MainEvents::RADIO_CHECK); 
    }

    //==================================================
}


// Command check 
void BoatRadio::CommandCheck(Boat& boat_radio)
{
    // Choose an action based on which data pipe the payload was received on. If the data 
    // pipe of the received payload doesn't match a pre-defined pipe then do nothing with 
    // the data. 

    // 'gs_connect_timeout', 'gs_connect_flag', 'data_pipe' and 'read_buff' are not 
    // protected because 'CommandRead' and 'CommandCheck' are only called sequentially 
    // every 100ms so there is no threat of one interrupting the other. 

    // Ground station data pipe 
    if (data_pipe == gs_pipe)
    {
        // If the payload matches one of the ground station commands then set the 
        // connection flag and reset the timeout. This helps the boat know if it has 
        // a radio connection to the ground station. 
        if (CommandLookUp(read_buff, command_table, boat_radio))
        {
            gs_connect_flag = SET_BIT; 
            gs_connect_timeout = CLEAR; 
        }
    }
}


// Command set 
void BoatRadio::CommandSet(
    Boat& boat_radio, 
    const char *command)
{
    // 'write_ptr' is not protected because 'CommandSend' is only called after this 
    // function. 

    write_ptr = (uint8_t *)command; 
    boat_radio.CommsEventQueue((Event)Boat::CommsEvents::RADIO_SEND); 
}


// Command send 
void BoatRadio::CommandSend(void)
{
    // 'write_ptr' is not protected because this function is only called after 
    // 'CommandSet' has been called. 

    nrf24l01_send_payload(write_ptr); 
}


// Radio connection status 
uint8_t BoatRadio::ConnectionStatus(void)
{
    return gs_connect_flag; 
}

//=======================================================================================


//=======================================================================================
// Command callbacks 

// Heartbeat command 
void BoatRadio::HBCmd(
    Boat& boat_radio, 
    uint8_t *hb_cmd_arg)
{
    // This function is here to provide a match for a valid command check. It's meant to 
    // verify that there is a radio connection but the connection flag gets set when any 
    // command matches. 

    boat_radio.radio.CommandSet(boat_radio, vehicle_radio_ping_confirm); 
}


// Idle (standby) state command 
void BoatRadio::IdleCmd(
    Boat& boat_radio, 
    uint8_t *idle_cmd_arg)
{
    boat_radio.main_flags.standby_state = SET_BIT; 
    boat_radio.MainStateChange(); 
    boat_radio.radio.CommandSet(boat_radio, vehicle_radio_msg_confirm); 
}


// Autonomous state command 
void BoatRadio::AutoCmd(
    Boat& boat_radio, 
    uint8_t *auto_cmd_arg)
{
    // Check for GPS connection before triggering a state change 
    if (boat_radio.navigation.navstat)
    {
        boat_radio.main_flags.auto_state = SET_BIT; 
        boat_radio.MainStateChange(); 
        boat_radio.radio.CommandSet(boat_radio, vehicle_radio_msg_confirm); 
    }
}


// Manual control state command 
void BoatRadio::ManualCmd(
    Boat& boat_radio, 
    uint8_t *manual_cmd_arg)
{
    boat_radio.main_flags.manual_state = SET_BIT; 
    boat_radio.MainStateChange(); 
    boat_radio.radio.CommandSet(boat_radio, vehicle_radio_msg_confirm); 
}


// Waypoint mission index update command 
void BoatRadio::IndexCmd(
    Boat& boat_radio, 
    uint8_t *index_cmd_arg)
{
    if (index_cmd_arg == nullptr)
    {
        return; 
    }

    // Attempt to update the target waypoint with the index from the command. If 
    // successful then send a confirmation back to the ground station. 
    if (boat_radio.navigation.TargetUpdate(*index_cmd_arg))
    {
        boat_radio.radio.CommandSet(boat_radio, vehicle_radio_msg_confirm); 
    }
}


// Manual throttle command 
void BoatRadio::ThrottleCmd(
    Boat& boat_radio, 
    uint8_t *throttle_cmd_arg)
{
    // Call the manual control function. We pass the radio command parameters to be used 
    // as thruster control because they've already been verified by the radio command 
    // check. 
    boat_radio.rc.RemoteControl(
        (int16_t)boat_radio.radio.cmd_value,    // Throttle requested 
        boat_radio.radio.cmd_id[0],             // Motor ID 
        boat_radio.radio.cmd_id[1]);            // Direction ID 

    // Send/queue a response to the ground station 
}

//=======================================================================================


//=======================================================================================
// Command enable/disable 

// Main thread: Standby state 
void BoatRadio::MainStandbyStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(vehicle_radio_cmd_ping, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_auto, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_manual, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_index, command_table, cmd_state); 
}


// Main thread: Auto state 
void BoatRadio::MainAutoStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(vehicle_radio_cmd_ping, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_idle, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_index, command_table, cmd_state); 
}


// Main thread: Manual state 
void BoatRadio::MainManualStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(vehicle_radio_cmd_ping, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_idle, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_RP, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_RN, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_RM, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_LP, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_LN, command_table, cmd_state); 
    CommandEnable(vehicle_radio_cmd_LM, command_table, cmd_state); 
}


// Main thread: Low Power state 
void BoatRadio::MainLowPwrStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(vehicle_radio_cmd_ping, command_table, cmd_state); 
}


// Main thread: Fault state 
void BoatRadio::MainFaultStateCmdEnable(uint8_t cmd_state)
{
    CommandEnable(vehicle_radio_cmd_ping, command_table, cmd_state); 
}

//=======================================================================================
