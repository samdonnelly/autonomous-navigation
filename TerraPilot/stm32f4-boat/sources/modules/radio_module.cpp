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
#include "gs_interface_config.h" 
#include "vehicle_radio_config.h" 

//=======================================================================================


//=======================================================================================
// Instantiate the template for its use cases 

template class RadioModule<GroundStation, GS_NUM_CMDS>; 
template class RadioModule<Boat, VEHICLE_RADIO_NUM_CMDS>; 

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

    uint8_t buff_index = CLEAR; 
    uint8_t lookup_status = FALSE; 

    // Parse the ID from the command. Only proceed if the ID has a valid format. 
    if (IDParse(cmd_buff, buff_index))
    {
        // Look for match to the ID. If a match is found then asses additional command 
        // arguments. 

        uint8_t num_cmds = (uint8_t)cmd_table.size(); 

        for (uint8_t i = CLEAR; i < num_cmds; i++)
        {
            if (cmd_table[i].cmd_enable)
            {
                if (strcmp((char *)cmd_id, cmd_table[i].cmd) == 0)
                {
                    // ID matches an enabled pre-defined command. Check if the 
                    // command contains a value or string argument then call the 
                    // command callback. 

                    cmd_value = CLEAR; 
                    memset((void *)cmd_str, NULL_CHAR, sizeof(cmd_str)); 

                    switch (cmd_table[i].cmd_arg_type)
                    {
                        case CMD_ARG_VALUE: 
                            if (ValueParse(cmd_buff, buff_index))
                            {
                                cmd_table[i].cmd_func_ptr(vehicle, &cmd_value); 
                                lookup_status = TRUE; 
                            }
                            break; 

                        case CMD_ARG_STR: 
                            StringParse(cmd_buff, buff_index); 
                            cmd_table[i].cmd_func_ptr(vehicle, cmd_str); 
                            lookup_status = TRUE; 
                            break; 
                        
                        default:   // No argument command 
                            cmd_table[i].cmd_func_ptr(vehicle, nullptr); 
                            lookup_status = TRUE; 
                            break; 
                    }

                    break; 
                }
            }
        }
    }

    return lookup_status; 
}


// Enable/Disable the specified command 
template <typename C, size_t SIZE> 
void RadioModule<C, SIZE>::CommandEnable(
    const char *cmd, 
    std::array<RadioCmdData, SIZE>& cmd_table, 
    uint8_t cmd_state) const 
{
    uint8_t num_cmds = (uint8_t)cmd_table.size(); 

    for (uint8_t i = CLEAR; i < num_cmds; i++)
    {
        if (strcmp(cmd, cmd_table[i].cmd) == 0)
        {
            cmd_table[i].cmd_enable = cmd_state; 
            break; 
        }
    }
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Parse ID from the command 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::IDParse(
    const uint8_t *cmd_buff, 
    uint8_t& buff_index)
{
    uint8_t id_status = TRUE; 
    uint8_t data = cmd_buff[buff_index]; 

    // Parse the ID from the command 
    while ((data != NULL_CHAR) && (data != SPACE_CHAR))
    {
        // Check that the command byte is within range 
        if (((data >= A_LO_CHAR) && (data <= Z_LO_CHAR)) || 
            ((data >= A_UP_CHAR) && (data <= Z_UP_CHAR)) || 
            ((data >= ZERO_CHAR) && (data <= NINE_CHAR)) || 
            (data == UNDERSCORE_CHAR))
        {
            // Valid character byte seen 
            cmd_id[buff_index] = data; 
        }
        else 
        {
            // Valid data not seen 
            id_status = FALSE; 
            break; 
        }

        data = cmd_buff[++buff_index]; 
    }

    cmd_id[buff_index++] = NULL_CHAR; 

    return id_status; 
}


// Parse value from the command 
template <typename C, size_t SIZE> 
uint8_t RadioModule<C, SIZE>::ValueParse(
    const uint8_t *cmd_buff, 
    uint8_t& buff_index)
{
    uint8_t data = cmd_buff[buff_index]; 
    uint8_t value_index = CLEAR; 
    uint8_t cmd_value_str[MAX_RADIO_CMD_SIZE]; 

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

    return TRUE; 
}


// Parse string argument from the command 
template <typename C, size_t SIZE> 
void RadioModule<C, SIZE>::StringParse(
    const uint8_t *cmd_buff, 
    uint8_t& buff_index)
{
    uint8_t data = cmd_buff[buff_index]; 

    for (uint8_t i = CLEAR; data != NULL_CHAR; i++)
    {
        cmd_str[i] = data; 
        data = cmd_buff[++buff_index]; 
    }
}

//=======================================================================================
