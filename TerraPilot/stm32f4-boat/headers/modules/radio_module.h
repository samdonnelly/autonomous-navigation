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
#include <array> 

//=======================================================================================


//=======================================================================================
// Macros 

#define MAX_RADIO_CMD_SIZE 32 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare the classes that will use the radio module 
class GroundStation; 
class Boat; 

// Radio base class 
template <typename C, size_t SIZE> 
class RadioModule 
{
protected:   // Protected members 

    // Command payload data (<ID> <value> or <ID> <string>). The size of the these 
    // buffers must be able to accomodate anything the radio can send. This will depend 
    // on the device being used. 
    uint8_t cmd_id[MAX_RADIO_CMD_SIZE];     // ID from the command 
    uint8_t cmd_value;                      // Value from the command 
    uint8_t cmd_str[MAX_RADIO_CMD_SIZE];    // String of the command 

    // Command argument types 
    enum RadioCmdArgs 
    {
        CMD_ARG_NONE, 
        CMD_ARG_VALUE, 
        CMD_ARG_STR 
    }; 

    // Command control 
    struct RadioCmdData 
    {
        const char *cmd; 
        RadioCmdArgs cmd_arg_type; 
        void (*cmd_func_ptr)(C&, uint8_t*); 
        uint8_t cmd_enable; 
    }; 

private:   // Private member functions 

    // Parse ID from the command 
    uint8_t IDParse(
        const uint8_t *cmd_buff, 
        uint8_t& buff_index); 

    // Parse value argument from the command 
    uint8_t ValueParse(
        const uint8_t *cmd_buff, 
        uint8_t& buff_index); 

    // Parse string argument from the command 
    void StringParse(
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
        const char *cmd, 
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
