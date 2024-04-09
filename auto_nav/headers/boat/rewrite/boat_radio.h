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
// Macros 

#define BOAT_RADIO_NUM_CMDS 9 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare Boat class 
class Boat; 

class BoatRadio : public RadioModule 
{
private:   // Private members 

    nrf24l01_data_pipe_t pipe;               // Data pipe number for the radio module 
    uint8_t read_buff[MAX_RADIO_CMD_SIZE];   // Data read by PRX from PTX device 
    uint8_t connect_flag;                    // Connect flag 

private:   // Private member functions 

    //==================================================
    // Commands 

    // Command callbacks 
    static void IdleCmd(Boat& boat_radio, uint8_t idle_cmd_value); 
    static void AutoCmd(Boat& boat_radio, uint8_t auto_cmd_value); 
    static void ManualCmd(Boat& boat_radio, uint8_t manual_cmd_value); 
    static void IndexCmd(Boat& boat_radio, uint8_t index_cmd_value); 
    static void ThrottleCmd(Boat& boat_radio, uint8_t throttle_cmd_value); 
    static void HBCmd(Boat& boat_radio, uint8_t hb_cmd_value); 

    // Command table 
    RadioCmdData<Boat> cmd_table[BOAT_RADIO_NUM_CMDS] = 
    {
        {"idle",   &IdleCmd,     CLEAR_BIT}, 
        {"auto",   &AutoCmd,     CLEAR_BIT}, 
        {"manual", &ManualCmd,   CLEAR_BIT}, 
        {"index",  &IndexCmd,    CLEAR_BIT},  
        {"RP",     &ThrottleCmd, CLEAR_BIT}, 
        {"RN",     &ThrottleCmd, CLEAR_BIT}, 
        {"LP",     &ThrottleCmd, CLEAR_BIT}, 
        {"LN",     &ThrottleCmd, CLEAR_BIT}, 
        {"ping",   &HBCmd,       CLEAR_BIT} 
    }; 

    // Test hash command table 
    std::unordered_map<std::string, RadioCmdData_test<Boat>> command_table[BOAT_RADIO_NUM_CMDS] = 
    {
        // {"idle",   {"idle",   &IdleCmd,     CLEAR_BIT} }, 
        // {"auto",   {"auto",   &AutoCmd,     CLEAR_BIT} }, 
        // {"manual", {"manual", &ManualCmd,   CLEAR_BIT} }, 
        // {"index",  {"index",  &IndexCmd,    CLEAR_BIT} },  
        // {"RP",     {"RP",     &ThrottleCmd, CLEAR_BIT} }, 
        // {"RN",     {"RN",     &ThrottleCmd, CLEAR_BIT} }, 
        // {"LP",     {"LP",     &ThrottleCmd, CLEAR_BIT} }, 
        // {"LN",     {"LN",     &ThrottleCmd, CLEAR_BIT} }, 
        // {"ping",   {"ping",   &HBCmd,       CLEAR_BIT} } 
    }; 
    
    //==================================================

public:   // Public member functions 

    // Constructor(s) 
    BoatRadio() 
        : RadioModule(BOAT_RADIO_NUM_CMDS) {} 

    // Destructor 
    ~BoatRadio() {} 

    // Command read 
    void CmdRead(void); 

    // Command enable/disable 
    void MainStandbyStateCmdEnable(uint8_t cmd_state); 
    void MainAutoStateCmdEnable(uint8_t cmd_state); 
    void MainManualStateCmdEnable(uint8_t cmd_state); 
    void MainLowPwrStateCmdEnable(uint8_t cmd_state); 
    void MainFaultStateCmdEnable(uint8_t cmd_state); 
}; 

//=======================================================================================

#endif   // _BOAT_RADIO_H_ 
