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

#include "tools.h" 

// Standard library 
#include <string> 
#include <array> 

//=======================================================================================


//=======================================================================================
// Macros 

#define MAX_RADIO_CMD_SIZE 32 

//=======================================================================================


//=======================================================================================
// Enums 

enum RadioCmdArgs 
{
    NRF24L01_CMD_ARG_NONE, 
    NRF24L01_CMD_ARG_VALUE, 
    NRF24L01_CMD_ARG_STR 
}; 

//=======================================================================================


//=======================================================================================
// Structs 

// User command data 
struct RadioCmdData 
{
    uint8_t cmd_buff[MAX_RADIO_CMD_SIZE];   // User command parsed from the CB 
    uint8_t cmd_id[MAX_RADIO_CMD_SIZE];     // ID from the user command 
    uint8_t cmd_value;                      // Value from the user command 
    uint8_t cmd_str[MAX_RADIO_CMD_SIZE];    // String from the user command 
}; 

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

    // Parse command into ID and value 
    uint8_t CommandParse(const uint8_t *cmd_buff); 

    // Parse ID from the command 
    uint8_t IDParse(
        const uint8_t *cmd_buff, 
        uint8_t& buff_index); 

    // Parse value from the command 
    uint8_t ValueParse(
        const uint8_t *cmd_buff, 
        uint8_t& buff_index); 

protected:   // Protected member functions 

    // Look for a matching command 
    uint8_t CommandLookUp(
        const uint8_t *cmd_buff, 
        std::array<RadioCmdData, SIZE>& cmd_table, 
        C& vehicle); 

    // Enable/Disable the specified command 
    void CommandEnable(
        const std::string& cmd, 
        std::array<RadioCmdData, SIZE>& cmd_table, 
        uint8_t cmd_state) const; 

public:   // Public member functions 

    // Constructor(s) 
    RadioModule() {} 

    // Destructor 
    ~RadioModule() {} 
}; 

//=======================================================================================

#endif   // _RADIO_MODULE_H_ 
