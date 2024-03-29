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

// Note that is FreeRTOS is not used then it has to be removed from CMakeLists 
#define FREERTOS_ENABLE 1 

// Use these to select which code to use and place them around interrupts 
#define GROUND_STATION 0 
#define BOAT 1 

//==================================================

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _SYSTEM_SETTINGS_H_ 
