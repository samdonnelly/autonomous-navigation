/**
 * @file m8q_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief SAM-M8Q GPS configuration file 
 * 
 * @version 0.1
 * @date 2022-10-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */

//=======================================================================================
// Includes 

#include "m8q_config.h"

//=======================================================================================


//=======================================================================================
// Variables 

// M8Q configuration messages 
static char* m8q_config_msgs[M8Q_CONFIG_MSG_NUM] = 
{
    // Disable default NMEA messages 
    "$PUBX,40,GGA,0,0,0,0,0,0*",    // GGA disable
    "$PUBX,40,GLL,0,0,0,0,0,0*",    // GLL disable
    "$PUBX,40,GSA,0,0,0,0,0,0*",    // GSA disable
    "$PUBX,40,GSV,0,0,0,0,0,0*",    // GSV disable
    "$PUBX,40,RMC,0,0,0,0,0,0*",    // RMC disable
    "$PUBX,40,VTG,0,0,0,0,0,0*",    // VTG disable 

    // UBX config messages  
    "B5,62,06,01,0800,F1,00,01,00,00,00,00,00*",      // POSITION enable 
    "B5,62,06,01,0800,F1,04,0A,00,00,00,00,00*",      // TIME enable 

    // Power configuration 
    "B5,62,06,3B,3000,02,00,00,00,60104201,E8030000,10270000,00000000,"
    "0000,0000,0000000000000000000000000000000000000000,00000000*",

    // Port configuration 
    "B5,62,06,00,1400,01,00,0000,C0080000,80250000,0000,0000,0000,0000*",
    "B5,62,06,00,1400,00,00,9902,84000000,00000000,0700,0300,0200,0000*", 

    // Save the settings (save mask) 
    "B5,62,06,09,0C00,00000000,FFFFFFFF,00000000*" 
};

//=======================================================================================


//=======================================================================================
// Functions 

// M8Q copy config messages 
void m8q_config_copy(char config_msgs[M8Q_CONFIG_MSG_NUM][M8Q_CONFIG_MSG_MAX_LEN])
{
    // Local variables 
    uint8_t msg_num = 0; 
    uint8_t msg_index = 0; 
    char msg_byte; 

    // Loop through all configuration messages 
    while (msg_num < M8Q_CONFIG_MSG_NUM)
    {
        msg_byte = m8q_config_msgs[msg_num][msg_index]; 

        // Copy a message 
        while (msg_byte != AST_CHAR)
        {
            config_msgs[msg_num][msg_index++] = msg_byte; 
            msg_byte = m8q_config_msgs[msg_num][msg_index]; 
        }

        // Terminate the message 
        config_msgs[msg_num][msg_index++] = CR_CHAR; 
        config_msgs[msg_num][msg_index] = NULL_CHAR; 
        
        // Increment to the next message and reset the message index 
        msg_num++; 
        msg_index = 0; 
    }
}

//=======================================================================================
