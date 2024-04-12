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

//=======================================================================================


//=======================================================================================
// Instantiate the template for its use cases 

// Boat 
template class RadioModule<Boat>; 

//=======================================================================================


//=======================================================================================
// Radio communication 

// Look for a matching command 
template <typename C> 
uint8_t RadioModule<C>::CommandLookUp(
    uint8_t *cmd_buff, 
    std::unordered_map<std::string, RadioCmdData>& cmd_table, 
    C& vehicle)
{
    return TRUE; 
}


// Command parse into ID and value 
template <typename C> 
uint8_t RadioModule<C>::CommandParse(uint8_t *cmd_buff)
{
    return TRUE; 
}

//=======================================================================================
