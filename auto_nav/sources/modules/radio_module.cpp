/**
 * @file radio_module.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Radio module 
 * 
 * @version 0.1
 * @date 2024-04-07
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
// User functions 

// Look for a matching command 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::CommandLookUp(
    uint8_t *cmd_buff, 
    std::array<RadioCmdData, SIZE>& cmd_table, 
    C& vehicle)
{
    if (cmd_buff == nullptr)
    {
        return FALSE; 
    }

    // // Parse the received command into an ID and value. 
    // if (CommandParse(cmd_buff))
    // {
    //     // The command is of a valid format. Check if the ID matches one of the available 
    //     // vehicle commands. 
    //     const auto& command_table = cmd_table; 
    //     std::string cmd_id_str = (char *)cmd_id; 

    //     if (command_table.find(cmd_id_str) != command_table.end())
    //     {
    //         // Command exists. Check if the command is enabled. If so then call the 
    //         // command callback function. 
    //         const RadioCmdData& command_data = command_table.at(cmd_id_str); 

    //         if (command_data.cmd_enable)
    //         {
    //             command_data.cmd_func_ptr(vehicle, cmd_value); 
    //             return TRUE; 
    //         }
    //     }
    // }

    return FALSE; 
}


// Enable/Disable the specified command 
template <typename C, size_t SIZE> 
void RadioModule<C, SIZE>::CommandEnable(
    std::string cmd, 
    std::array<RadioCmdData, SIZE>& cmd_table)
{
    // 
    for (uint8_t i = CLEAR; i < 10; i++)
    {
        if (cmd.compare(cmd_table[i].cmd) == 0)
        {
            // 
        }
    }
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Command parse into ID and value 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::CommandParse(uint8_t *cmd_buff)
{
    uint8_t id_flag = SET_BIT; 
    uint8_t id_index = CLEAR; 
    uint8_t data = CLEAR; 
    uint8_t cmd_value_str[MAX_RADIO_CMD_SIZE]; 
    uint8_t value_size = CLEAR; 

    // Initialize data 
    memset((void *)cmd_id, CLEAR, sizeof(cmd_id)); 
    cmd_value = CLEAR; 
    memset((void *)cmd_value_str, CLEAR, sizeof(cmd_value_str)); 

    // Parse the command into an ID and value 
    for (uint8_t i = CLEAR; cmd_buff[i] != NULL_CHAR; i++)
    {
        data = cmd_buff[i]; 

        if (id_flag)
        {
            // cmd ID parsing 

            id_index = i; 

            // Check that the command byte is within range 
            if ((data >= A_LO_CHAR && data <= Z_LO_CHAR) || 
                (data >= A_UP_CHAR && data <= Z_UP_CHAR))
            {
                // Valid character byte seen 
                cmd_id[i] = data; 
            }
            else if (data >= ZERO_CHAR && data <= NINE_CHAR)
            {
                // Valid digit character byte seen 
                id_flag = CLEAR_BIT; 
                cmd_id[i] = NULL_CHAR; 
                cmd_value_str[i-id_index] = data; 
                value_size++; 
            }
            else 
            {
                // Valid data not seen 
                return FALSE; 
            }
        }
        else 
        {
            // cmd value parsing 

            if (data >= ZERO_CHAR && data <= NINE_CHAR)
            {
                // Valid digit character byte seen 
                cmd_value_str[i-id_index] = data; 
                value_size++; 
            }
            else 
            {
                // Valid data not seen 
                return FALSE; 
            }
        }
    }

    // Calculate the cmd value 
    for (uint8_t i = CLEAR; i < value_size; i++)
    {
        cmd_value += (uint8_t)char_to_int(cmd_value_str[i], value_size-i-1); 
    }

    return TRUE; 
}

//=======================================================================================
