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
    const uint8_t *cmd_buff, 
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
                if (cmd_id_str.compare(cmd_table[i].cmd) == 0)
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
    uint8_t cmd_state) const 
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

// Parse command into ID and value 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::CommandParse(const uint8_t *cmd_buff)
{
    // Command format: 
    // <ID><space><value> 

    uint8_t buff_index = 0; 

    memset((void *)cmd_id, NULL_CHAR, sizeof(cmd_id)); 
    cmd_value = CLEAR; 

    if (IDParse(cmd_buff, buff_index))
    {
        if (ValueParse(cmd_buff, buff_index)) 
        {
            return TRUE; 
        }
    }

    return FALSE; 


    //==================================================
    // Dev 

    // nrf24l01_cmd_arg_t cmd_arg_flag = NRF24L01_CMD_ARG_NONE; 
    // uint8_t id_index = CLEAR; 
    // uint8_t data = cmd_data->cmd_buff[0]; 
    // uint8_t cmd_value_buff[NRF24L01_TEST_MAX_INPUT]; 
    // uint8_t value_size = CLEAR; 

    // // Initialize data 
    // memset((void *)cmd_id, NULL_CHAR, sizeof(cmd_id)); 
    // cmd_value = CLEAR; 
    // memset((void *)cmd_str, CLEAR, sizeof(cmd_str)); 
    // memset((void *)cmd_value_buff, CLEAR, sizeof(cmd_value_buff)); 

    // // Parse the command into an ID and value 
    // for (uint8_t i = CLEAR; data != NULL_CHAR; i++)
    // {
    //     switch (cmd_arg_flag)
    //     {
    //         case NRF24L01_CMD_ARG_NONE:   // Command ID parsing 

    //             // Check that the command byte is within range 
    //             if ((data >= A_LO_CHAR && data <= Z_LO_CHAR) || 
    //                 (data >= A_UP_CHAR && data <= Z_UP_CHAR) || 
    //                 (data >= ZERO_CHAR && data <= NINE_CHAR) || 
    //                 (data == UNDERSCORE_CHAR))
    //             {
    //                 // Valid character byte seen 
    //                 cmd_id[i] = data; 
    //             }
    //             else if (data == SPACE_CHAR)
    //             {
    //                 // End of ID, start of optional value 
    //                 cmd_arg_flag = cmd_arg_type; 
    //                 cmd_id[i] = NULL_CHAR; 
    //                 id_index = i + 1; 
    //             }
    //             else 
    //             {
    //                 // Valid data not seen 
    //                 return FALSE; 
    //             }

    //             break; 

    //         case NRF24L01_CMD_ARG_VALUE:   // Command value parsing 

    //             if (data >= ZERO_CHAR && data <= NINE_CHAR)
    //             {
    //                 // Valid digit character byte seen 
    //                 cmd_value_buff[i-id_index] = data; 
    //                 value_size++; 
    //             }
    //             else 
    //             {
    //                 // Valid data not seen 
    //                 return FALSE; 
    //             }

    //             break; 

    //         case NRF24L01_CMD_ARG_STR:   // Command string parsing 
    //             cmd_data->cmd_str[i-id_index] = data; 
    //             break; 

    //         default: 
    //             break; 
    //     }

    //     data = cmd_data->cmd_buff[i+1]; 
    // }

    // // Calculate the cmd value 
    // for (uint8_t i = CLEAR; i < value_size; i++)
    // {
    //     cmd_value += (uint8_t)char_to_int(cmd_value_buff[i], value_size-i-1); 
    // }

    // return TRUE; 

    //==================================================
}


// Parse ID from the command 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::IDParse(
    const uint8_t *cmd_buff, 
    uint8_t& buff_index)
{
    uint8_t data = cmd_buff[buff_index]; 

    while ((data != NULL_CHAR) && (data != SPACE_CHAR))
    {
        if (((data >= A_LO_CHAR) && (data <= Z_LO_CHAR)) || 
            ((data >= A_UP_CHAR) && (data <= Z_UP_CHAR)) || 
            ((data >= ZERO_CHAR) && (data <= NINE_CHAR)))
        {
            cmd_id[buff_index] = data; 
        }
        else 
        {
            // Invalid character 
            return FALSE; 
        }

        data = cmd_buff[++buff_index]; 
    }

    return TRUE; 
}


// Parse value from the command 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::ValueParse(
    const uint8_t *cmd_buff, 
    uint8_t& buff_index)
{
    uint8_t data = cmd_buff[buff_index], value_index = CLEAR; 
    uint8_t cmd_value_str[MAX_RADIO_CMD_SIZE]; 

    if (data != NULL_CHAR)
    {
        data = cmd_buff[++buff_index]; 

        while ((data != NULL_CHAR) && (data != SPACE_CHAR))
        {
            if ((data >= ZERO_CHAR) && (data <= NINE_CHAR))
            {
                cmd_value_str[value_index++] = data; 
            }
            else 
            {
                // Invalid digit character 
                return FALSE; 
            }

            data = cmd_buff[++buff_index]; 
        }

        // Convert to number 
        for (uint8_t i = CLEAR; i < value_index; i++)
        {
            cmd_value += (uint8_t)char_to_int(cmd_value_str[i], value_index-i-1); 
        }
    }

    return TRUE; 
}

//=======================================================================================
