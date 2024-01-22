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

// Number of messages in a configuration packet 
#define M8Q_CONFIG_MSG_NUM 12 

// Max length of a single config message in a packet 
#define M8Q_CONFIG_MSG_MAX_LEN 150 

//=======================================================================================


//=======================================================================================
// Config messages 

extern const char m8q_config_msgs[M8Q_CONFIG_MSG_NUM][M8Q_CONFIG_MSG_MAX_LEN]; 

//=======================================================================================

#endif  // _M8Q_CONFIG_H_