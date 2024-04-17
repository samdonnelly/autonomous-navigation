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
#include "boat_radio_config.h" 

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

    // Parse the command into an ID and value 
    if (CommandParse(cmd_buff))
    {
        // The command has a valid format. Check if the ID matches one of the enabled 
        // predefined commands. If so then call the command callback function. 

        uint8_t num_cmds = (uint8_t)cmd_table.size(); 
        std::string cmd_id_str = (char *)cmd_id; 

        for (uint8_t i = CLEAR; i < num_cmds; i++)
        {
            if (cmd_table[i].cmd_enable)
            {
                if (cmd_id_str.compare(cmd_table[i].cmd))
                {
                    cmd_table[i].cmd_func_ptr(vehicle, cmd_value); 
                    return TRUE; 
                }
            }
        }
    }

    return FALSE; 
}


// Enable/Disable the specified command 
template <typename C, size_t SIZE> 
void RadioModule<C, SIZE>::CommandEnable(
    const std::string& cmd, 
    std::array<RadioCmdData, SIZE>& cmd_table, 
    uint8_t cmd_state)
{
    uint8_t num_cmds = (uint8_t)cmd_table.size(); 

    for (uint8_t i = CLEAR; i < num_cmds; i++)
    {
        if (cmd.compare(cmd_table[i].cmd) == 0)
        {
            cmd_table[i].cmd_enable = cmd_state; 
            break; 
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

    // Command format: 
    // <ID><space><value> 
    // - Number of spaces between ID and value does not matter 
    // - Number of spaces after value does not matter 
    // - Call the ID parse function that reads until a space or NULL --> seeing only 
    //   characters then a space is valid 
    // - If the ID part is valid then call the value parse function to read until a space 
    //   or NULL --> seeing only numbers (or no number) and then a space or NULL is valid 
    // 

    // 
    for (uint8_t i = CLEAR; cmd_buff[i] != NULL_CHAR; i++); 

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

                // id_index = i;   // Remove this line from up top 
                // cmd_value_str[0] = data; 
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

            // If there is no number but a valid ID format then this part of the code will 
            // not be reached but the parse will still return true. 

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
