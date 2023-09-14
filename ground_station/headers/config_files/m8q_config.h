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

#ifndef _M8Q_CONFIG_H_ 
#define _M8Q_CONFIG_H_ 

//=======================================================================================
// Includes 

#include "stm32f411xe.h"
#include "tools.h"

//=======================================================================================


//=======================================================================================
// Macros 

#define M8Q_CONFIG_MSG_NUM 12        // Number of M8Q configuration messages on startup 
#define M8Q_CONFIG_MSG_MAX_LEN 150   // Maximum length of an M8Q configuration message 

//=======================================================================================


//=======================================================================================
// Function prototypes 

/**
 * @brief M8Q configuration message copy 
 * 
 * @details This function copies the configuration messages defined in the m8q_config 
 *          file into the array passed to this function. This array can then be passed to 
 *          the m8q_init function to configure the receiver. 
 *          
 *          The messages defined in the config file are meant to configure the settings/
 *          behavior of the receiver. The M8Q doesn't have flash memory so it must be 
 *          reconfigured every time it looses power and battery backup power. These 
 *          messages are carefully constructed according to the message format in the 
 *          devices protocol datasheet. 
 *          
 *          The configuration messages are defined in a separate config file and copied 
 *          into an array so that there is a central location to define the messages 
 *          whether they are used in a project or not. 
 * 
 * @param config_msgs 
 */
void m8q_config_copy(char config_msgs[M8Q_CONFIG_MSG_NUM][M8Q_CONFIG_MSG_MAX_LEN]); 

//=======================================================================================

#endif  // _M8Q_CONFIG_H_