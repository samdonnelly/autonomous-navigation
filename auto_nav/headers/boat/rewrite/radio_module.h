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

#include <string> 
#include <unordered_map> 

//=======================================================================================


//=======================================================================================
// Macros 

#define MAX_RADIO_CMD_SIZE 32 

//=======================================================================================


//=======================================================================================
// Structs 

// Resources: 
// - https://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key 
// - https://en.cppreference.com/w/cpp/utility/hash 

// Command identification 
template <class C> 
struct RadioCmdData_test 
{
    const std::string cmd[MAX_RADIO_CMD_SIZE]; 
    void (*cmd_func_ptr)(C&, uint8_t); 
    uint8_t cmd_enable; 
}; 


// Hash function 
template<class C> 
struct std::hash<RadioCmdData_test<C>>
{
    size_t operator()(const RadioCmdData_test<C>& cmd_data)
    {
        return std::hash<std::string>()(cmd_data.cmd); 
    }
}; 


// Command identification 
template <class C> 
struct RadioCmdData 
{
    const char cmd[MAX_RADIO_CMD_SIZE]; 
    void (*cmd_func_ptr)(C&, uint8_t); 
    uint8_t cmd_enable; 
}; 

//=======================================================================================


//=======================================================================================
// Classes 

class RadioModule 
{
private:   // Private members 
    
    uint8_t num_commands; 

protected:   // Protected members 

    // Payload data 
    uint8_t cmd_id[MAX_RADIO_CMD_SIZE];         // Stores the ID of the external command 
    uint8_t cmd_value;  

private:   // Private member functions 

    // Command parse into ID and value 
    uint8_t CmdParse(uint8_t *cmd_buff); 

public:   // Public member functions 

    // Constructor(s) 
    RadioModule(uint8_t num_cmds) 
        : num_commands(num_cmds) {} 

    // Destructor 
    ~RadioModule() {} 

    // Command check 
    template <class C> 
    uint8_t CmdCheck(
        uint8_t *cmd_buff, 
        RadioCmdData<C>& cmd_table, 
        C& vehicle); 
}; 

//=======================================================================================

#endif   // _RADIO_MODULE_H_ 
