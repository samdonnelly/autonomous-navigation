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
constexpr uint16_t s_to_ms = 1000;   // Seconds to milliseconds 
constexpr float kph_to_mps = 3.6f;   // Kilometers/hour to meters/second 

//=======================================================================================


//=======================================================================================
// Enums 

typedef enum {
    FLAG_CLEAR, 
    FLAG_SET
} flag_setter_t; 

//=======================================================================================

#endif   // _SYSTEM_TOOLS_H_ 
