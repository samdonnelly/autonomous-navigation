/**
 * @file system_settings.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System configuration settings 
 * 
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _SYSTEM_SETTINGS_H_ 
#define _SYSTEM_SETTINGS_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Macros 

//==================================================
// Conditional compilation 

// This needs to be set if using FreeRTOS as it includes additional files and alters 
// some core code that's used with FreeRTOS. Note that the 'RTOS_ENABLE' variable in 
// CMakeLists must be updated to match this macro. 
#define FREERTOS_ENABLE 0 

// Use these to select which code to use and place them around interrupts 
#define GROUND_STATION 1 
#define BOAT 0 

//==================================================

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _SYSTEM_SETTINGS_H_ 
