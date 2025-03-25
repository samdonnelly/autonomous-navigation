/**
 * @file hardware_config_template.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System configuration template 
 * 
 * @details This file is a template and is not to be altered unless there is a system 
 *          configuration setting being added or removed. This means these settings should 
 *          not be used to configure your system. Instead, create a "hardware_config.h" file 
 *          in the same folder as this file and redefine each of the macros below (i.e. 
 *          copy the contents of this file) and change those to configure your system. 
 *          "hardware_config.h" is not tracked so it's specific to your use case, but it's 
 *          included in the build path. This file is tracked but excluded from the build 
 *          path. 
 * 
 * @version 0.1
 * @date 2025-02-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _HARDWARE_CONFIG_TEMPLATE_H_ 
#define _HARDWARE_CONFIG_TEMPLATE_H_ 

//=======================================================================================
// Mode configuration 

// This needs to be set if using FreeRTOS as it includes additional files and alters 
// some core code that's used with FreeRTOS. Note that the 'RTOS_ENABLE' variable in 
// CMakeLists must be updated to match this macro. 
#define FREERTOS_ENABLE 0 

// Use these to select which code to use and place them around interrupts 
#define GROUND_STATION 0 
#define BOAT 0 

// To be removed 
#define NUM_GPS_WAYPOINTS 0 

//=======================================================================================


//=======================================================================================
// Hardware configuration 

// STM32F4 board selection 
// The below list allows for choosing which STM32F4 board to use. Redefine each of the 
// following in the "hardware_config.h" file and change your selected board to 1. 
#define STM32F4_05xx 0 
#define STM32F4_15xx 0 
#define STM32F4_07xx 0 
#define STM32F4_17xx 0 
#define STM32F4_27xx 0 
#define STM32F4_37xx 0 
#define STM32F4_29xx 0 
#define STM32F4_39xx 0 
#define STM32F4_01xC 0 
#define STM32F4_01xE 0 
#define STM32F4_10Tx 0 
#define STM32F4_10Cx 0 
#define STM32F4_10Rx 0 
#define STM32F4_11xE 0 
#define STM32F4_46xx 0 
#define STM32F4_69xx 0 
#define STM32F4_79xx 0 
#define STM32F4_12Cx 0 
#define STM32F4_12Zx 0 
#define STM32F4_12Rx 0 
#define STM32F4_12Vx 0 
#define STM32F4_13xx 0 
#define STM32F4_23xx 0 

//=======================================================================================

#endif   // _HARDWARE_CONFIG_TEMPLATE_H_ 
