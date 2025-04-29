/**
 * @file includes.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Includes interface 
 * 
 * @version 0.1
 * @date 2025-03-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _INCLUDES_H_ 
#define _INCLUDES_H_ 

//=======================================================================================
// Includes 

#include "system_tools.h" 
#include "rtos.h" 
#include "system_config.h" 

extern "C"
{
    // For C headers without C++ guards 
    #include "common/mavlink.h" 
}

//=======================================================================================

#endif   // _INCLUDES_H_ 
