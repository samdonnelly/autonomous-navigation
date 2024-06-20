/**
 * @file ground_station_radio_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station radio configuration interface 
 * 
 * @version 0.1
 * @date 2024-06-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _GROUND_STATION_RADIO_CONFIG_H_ 
#define _GROUND_STATION_RADIO_CONFIG_H_ 

//=======================================================================================
// Includes 
//=======================================================================================


//=======================================================================================
// Macros 

// Thruster command IDs 
#define GS_RADIO_LEFT_JOYSTICK 0x4C    // "L" character that indicates left joystick 
#define GS_RADIO_RIGHT_JOYSTICK 0x52   // "R" character that indicates right joystick 
#define GS_RADIO_FWD_DIRECTION 0x50    // "P" (plus) - indicates forward direction 
#define GS_RADIO_REV_DIRECTION 0x4D    // "M" (minus) - indicates reverse direction 

//=======================================================================================


//=======================================================================================
// Incoming commands 

#define GS_RADIO_NUM_CMDS 6 

extern const char 
// User commands 
gs_radio_cmd_manual[],       // 0. Manual control mode 
gs_radio_cmd_rf_channel[],   // 1. RF channel set 
gs_radio_cmd_rf_power[],     // 2. RF power set 
gs_radio_cmd_rf_dr[],        // 3. RF data rate set 
gs_radio_cmd_rf_dp[],        // 4. RF data pipe set 
gs_radio_cmd_update[];       // 5. Update serial terminal output 

//=======================================================================================


//=======================================================================================
// Outgoing messages 

extern const char 
gs_radio_confirm[];   // 

//=======================================================================================

#endif   // _GROUND_STATION_RADIO_CONFIG_H_ 
