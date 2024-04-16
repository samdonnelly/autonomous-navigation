/**
 * @file radio_module.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Radio module interface 
 * 
 * @version 0.1
 * @date 2024-04-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _RADIO_MODULE_H_
#define _RADIO_MODULE_H_ 

//=======================================================================================
// Includes 

#include "includes_drivers.h" 

// Standard library 
#include <string> 
#include <array> 

//=======================================================================================


//=======================================================================================
// Macros 

#define MAX_RADIO_CMD_SIZE 32 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare the classes that will use the radio module 
class Boat; 

// Radio base class 
template <typename C, size_t SIZE> 
class RadioModule 
{
protected:   // Protected members 

    // Command payload data 
    uint8_t cmd_id[MAX_RADIO_CMD_SIZE];         // Stores the ID of the external command 
    uint8_t cmd_value;  

    // Command control 
    struct RadioCmdData 
    {
        const std::string cmd; 
        void (*cmd_func_ptr)(C&, uint8_t); 
        uint8_t cmd_enable; 
    }; 

private:   // Private member functions 

    // Command parse into ID and value 
    uint8_t CommandParse(uint8_t *cmd_buff); 

protected:   // Protected member functions 

    // Look for a matching command 
    uint8_t CommandLookUp(
        uint8_t *cmd_buff, 
        std::array<RadioCmdData, SIZE>& cmd_table, 
        C& vehicle); 

    // Enable/Disable the specified command 
    void CommandEnable(
        std::string cmd, 
        std::array<RadioCmdData, SIZE>& cmd_table); 

public:   // Public member functions 

    // Constructor(s) 
    RadioModule() {} 

    // Destructor 
    ~RadioModule() {} 
}; 

//=======================================================================================

#endif   // _RADIO_MODULE_H_ 
