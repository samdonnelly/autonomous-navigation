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
private:   // Private member functions 

    //==================================================
    // Commands 

    // Command callbacks 
    static void IdleCmd(Boat& boat_radio, uint8_t idle_cmd_value); 
    static void ManualCmd(Boat& boat_radio, uint8_t manual_cmd_value); 
    static void AutoCmd(Boat& boat_radio, uint8_t auto_cmd_value); 
    static void IndexCmd(Boat& boat_radio, uint8_t index_cmd_value); 
    static void ThrottleCmd(Boat& boat_radio, uint8_t throttle_cmd_value); 
    static void HBCmd(Boat& boat_radio, uint8_t hb_cmd_value); 

    // Command table 
    RadioCmdData<Boat> cmd_table[BOAT_RADIO_NUM_CMDS] = 
    {
        {"idle",   &IdleCmd}, 
        {"manual", &ManualCmd}, 
        {"auto",   &AutoCmd}, 
        {"index",  &IndexCmd},  
        {"RP",     &ThrottleCmd}, 
        {"RN",     &ThrottleCmd}, 
        {"LP",     &ThrottleCmd}, 
        {"LN",     &ThrottleCmd}, 
        {"ping",   &HBCmd} 
    }; 
    
    //==================================================

public:   // Public member functions 

    // Constructor(s) 
    BoatRadio() {} 

    // Destructor 
    ~BoatRadio() {} 

    // Command read 
    void CmdRead(void); 
}; 

//=======================================================================================

#endif   // _BOAT_RADIO_H_ 
