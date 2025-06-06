/**
 * @file system_tools.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System tools interface 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _SYSTEM_TOOLS_H_ 
#define _SYSTEM_TOOLS_H_ 

//=======================================================================================
// Includes 

// C library 
#include <stdint.h> 
#include <string.h> 

// C++ library 
#include <array> 

//=======================================================================================


//=======================================================================================
// Macros 

// Data resets 
#define CLEAR_EVENT 0 
#define CLEAR_SETTING 0 
#define RESET 0 

// Units 
#define S_TO_MS 1000    // Seconds to milliseconds 

//=======================================================================================


//=======================================================================================
// Enums 

typedef enum {
    FLAG_CLEAR, 
    FLAG_SET
} flag_setter_t; 

//=======================================================================================

#endif   // _SYSTEM_TOOLS_H_ 
