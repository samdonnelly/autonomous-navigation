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

//=======================================================================================


//=======================================================================================
// Radio communication 

// Command check 
template <class C> 
uint8_t RadioModule::CmdCheck(
    uint8_t *cmd_buff, 
    RadioCmdData<C>& cmd_table, 
    C& vehicle)
{
    if (cmd_buff == nullptr)
    {
        return FALSE; 
    }

    // Validate the input - parse into an ID and value if valid 
    if (CmdParse(&cmd_buff[1]))
    {
        // Valid input - compare the ID to each of the available pre-defined commands 
        for (uint8_t i = CLEAR; i < num_commands; i++) 
        {
            // Check that the command is available for the state before comparing it 
            // against the ID. 
            if (cmd_table[i].cmd_enable)
            {
                // Command available. Compare with the ID. 
                if (str_compare(cmd_table[i].cmd, (char *)cmd_id, BYTE_0)) 
                {
                    // ID matched to a command. Execute the command. 
                    cmd_table[i].cmd_func_ptr(vehicle, cmd_value); 
                    return TRUE; 
                }
            }
        }
    }

    return FALSE; 
}


// Command parse into ID and value 
uint8_t RadioModule::CmdParse(uint8_t *cmd_buff)
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
