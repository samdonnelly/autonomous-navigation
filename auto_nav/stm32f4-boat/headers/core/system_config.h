/**
 * @file system_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System configuration 
 * 
 * @version 0.1
 * @date 2025-02-20
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _SYSTEM_CONFIG_H_ 
#define _SYSTEM_CONFIG_H_ 

//=======================================================================================
// Mode configuration 

#define FREERTOS_ENABLE 1 
#define GROUND_STATION 0 
#define BOAT 1 

#define NUM_GPS_WAYPOINTS 13 

//=======================================================================================


//=======================================================================================
// Hardware configuration 

// STM32F4 board selection 
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
#define STM32F4_11xE 1 
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

#endif   // _SYSTEM_CONFIG_H_ 
