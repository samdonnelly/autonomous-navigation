/**
 * @file radio_module_mock.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Radio module mock 
 * 
 * @version 0.1
 * @date 2024-04-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "radio_module.h" 
#include "boat_radio.h" 

//=======================================================================================


//=======================================================================================
// Instantiate the template for its use cases 

// Boat 
template class RadioModule<Boat, BOAT_RADIO_NUM_CMDS>; 

//=======================================================================================


//=======================================================================================
// Radio communication 

// Look for a matching command 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::CommandLookUp(
    uint8_t *cmd_buff, 
    std::array<RadioCmdData, SIZE>& cmd_table, 
    C& vehicle)
{
    return TRUE; 
}


// Command parse into ID and value 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::CommandParse(uint8_t *cmd_buff)
{
    return TRUE; 
}

//=======================================================================================
