/**
 * @file radio_comm.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB radio control interface 
 * 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _AB_RADIO_COMM_H_ 
#define _AB_RADIO_COMM_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "stm32f411xe.h" 

//=======================================================================================


//=======================================================================================
// Prototypes 

// /**
//  * @brief Parse the user command into an ID and value 
//  * 
//  * @details Takes a radio message received from the ground station and parses it into 
//  *          an ID and payload. If the ID and payload are of a valid format then the 
//  *          function will return true. Note that a payload is not needed for all 
//  *          commands. See the 'cmd_table' for a list of available commands/IDs and the 
//  *          states in which they're used. 
//  * 
//  * @param command_buffer : radio message string 
//  * @return uint8_t : status of the message parsing 
//  */
// uint8_t ab_parse_cmd(uint8_t *command_buffer); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_RADIO_COMM_H_ 
