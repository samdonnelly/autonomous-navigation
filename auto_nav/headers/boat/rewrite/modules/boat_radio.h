/**
 * @file boat_radio.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat radio interface 
 * 
 * @version 0.1
 * @date 2024-04-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_RADIO_H_
#define _BOAT_RADIO_H_ 

//=======================================================================================
// Includes 

#include "radio_module.h" 

//=======================================================================================


//=======================================================================================
// Messages/commands 

extern const std::string 
boat_radio_ping,           // 0. Ping (heartbeat) 
boat_radio_idle,           // 1. Idle (standby) state 
boat_radio_auto,           // 2. Autonomous state 
boat_radio_manual,         // 3. Manual (remote) control state 
boat_radio_index,          // 4. Waypoint index set 
boat_radio_RP,             // 5. Right thruster - forward thrust 
boat_radio_RN,             // 6. Right thruster - reverse thrust 
boat_radio_LP,             // 7. Left thruster - forward thrust 
boat_radio_LN;             // 8. Left thruster - reverse thrust 

//=======================================================================================


//=======================================================================================
// Classes 

class BoatRadio : public RadioModule<Boat> 
{
private:   // Private members 

    nrf24l01_data_pipe_t pipe;               // Data pipe number for the radio module 
    uint8_t read_buff[MAX_RADIO_CMD_SIZE];   // Data read by PRX from PTX device 
    uint8_t connect_flag;                    // Connect flag 

private:   // Private member functions 

    //==================================================
    // Commands 

    // Command callbacks 
    static void HBCmd(Boat& boat_radio, uint8_t hb_cmd_value); 
    static void IdleCmd(Boat& boat_radio, uint8_t idle_cmd_value); 
    static void AutoCmd(Boat& boat_radio, uint8_t auto_cmd_value); 
    static void ManualCmd(Boat& boat_radio, uint8_t manual_cmd_value); 
    static void IndexCmd(Boat& boat_radio, uint8_t index_cmd_value); 
    static void ThrottleCmd(Boat& boat_radio, uint8_t throttle_cmd_value); 

    // Command table 
    std::unordered_map<std::string, RadioCmdData> command_table = 
    {
        {boat_radio_ping,   { &HBCmd,       CLEAR_BIT} }, 
        {boat_radio_idle,   { &IdleCmd,     CLEAR_BIT} }, 
        {boat_radio_auto,   { &AutoCmd,     CLEAR_BIT} }, 
        {boat_radio_manual, { &ManualCmd,   CLEAR_BIT} }, 
        {boat_radio_index,  { &IndexCmd,    CLEAR_BIT} }, 
        {boat_radio_RP,     { &ThrottleCmd, CLEAR_BIT} }, 
        {boat_radio_RN,     { &ThrottleCmd, CLEAR_BIT} }, 
        {boat_radio_LP,     { &ThrottleCmd, CLEAR_BIT} }, 
        {boat_radio_LN,     { &ThrottleCmd, CLEAR_BIT} } 
    }; 
    
    //==================================================

public:   // Public member functions 

    // Constructor(s) 
    BoatRadio(nrf24l01_data_pipe_t data_pipe) 
        : pipe(data_pipe) {} 

    // Destructor 
    ~BoatRadio() {} 

    // Command read 
    void CommandRead(Boat& boat_radio); 

    // Command check 
    void CommandCheck(Boat& boat_radio); 

    // Command enable/disable 
    void MainStandbyStateCmdEnable(uint8_t cmd_state); 
    void MainAutoStateCmdEnable(uint8_t cmd_state); 
    void MainManualStateCmdEnable(uint8_t cmd_state); 
    void MainLowPwrStateCmdEnable(uint8_t cmd_state); 
    void MainFaultStateCmdEnable(uint8_t cmd_state); 
}; 

//=======================================================================================

#endif   // _BOAT_RADIO_H_ 
