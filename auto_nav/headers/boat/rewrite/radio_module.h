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

//=======================================================================================


//=======================================================================================
// Macros 

#define MAX_RADIO_CMD_SIZE 32 

//=======================================================================================


//=======================================================================================
// Structs 

// Ground station commands 
template <class C> 
struct RadioCmdData 
{
    const char cmd[MAX_RADIO_CMD_SIZE]; 
    void (*cmd_func_ptr)(C&, uint8_t); 
}; 

//=======================================================================================


//=======================================================================================
// Classes 

class RadioModule 
{
private:   // Private members 
    
    uint8_t num_commands; 
    uint16_t command_mask; 

public:   // Public members 

    // Payload data 
    uint8_t cmd_id[MAX_RADIO_CMD_SIZE];         // Stores the ID of the external command 
    uint8_t cmd_value;  

private:   // Private member functions 

    // Command parse into ID and value 
    uint8_t CmdParse(uint8_t *cmd_buff); 

public:   // Public member functions 

    // Constructor(s) 
    RadioModule() {} 

    // Destructor 
    ~RadioModule() {} 

    // Set command mask 
    void SetCmdMask(uint16_t mask); 

    // Command check 
    template <class C> 
    uint8_t CmdCheck(
        uint8_t *cmd_buff, 
        uint8_t state, 
        RadioCmdData<C>& cmd_table, 
        C& vehicle); 
}; 

//=======================================================================================

#endif   // _RADIO_MODULE_H_ 
