/**
 * @file commands_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System commands configuration interface 
 * 
 * @version 0.1
 * @date 2024-06-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _COMMANDS_CONFIG_H_ 
#define _COMMANDS_CONFIG_H_ 

//=======================================================================================
// Includes 
//=======================================================================================


//=======================================================================================
// Macros 

// Joystick position IDs 
#define GS_RADIO_LEFT_JOYSTICK 0x4C    // "L" character that indicates left joystick 
#define GS_RADIO_RIGHT_JOYSTICK 0x52   // "R" character that indicates right joystick 
#define GS_RADIO_FWD_DIRECTION 0x50    // "P" (plus) - indicates forward direction 
#define GS_RADIO_REV_DIRECTION 0x4D    // "M" (minus) - indicates reverse direction 

//=======================================================================================


//=======================================================================================
// User -> Ground Station commands 

#define GS_NUM_CMDS 6 

extern const char 
gs_cmd_manual[],       // 0. Manual control mode 
gs_cmd_rf_channel[],   // 1. RF channel set 
gs_cmd_rf_power[],     // 2. RF power set 
gs_cmd_rf_dr[],        // 3. RF data rate set 
gs_cmd_rf_dp[],        // 4. RF data pipe set 
gs_cmd_update[];       // 5. Update serial terminal output 

// Secondary commands 
extern const char 
gs_sub_cmd_on[],       // Enable 
gs_sub_cmd_off[];      // Disable 

//=======================================================================================


//=======================================================================================
// Ground Station -> Vehicle commands 

#define VEHICLE_RADIO_NUM_CMDS 9 

extern const char 
vehicle_radio_cmd_ping[],       // 0. Ping (heartbeat) 
vehicle_radio_cmd_idle[],       // 1. Idle (standby) state 
vehicle_radio_cmd_auto[],       // 2. Autonomous state 
vehicle_radio_cmd_manual[],     // 3. Manual (remote) control state 
vehicle_radio_cmd_index[],      // 4. Waypoint index set 
vehicle_radio_cmd_RP[],         // 5. Right joystick - forward direction 
vehicle_radio_cmd_RN[],         // 6. Right joystick - reverse direction 
vehicle_radio_cmd_LP[],         // 7. Left joystick - forward direction 
vehicle_radio_cmd_LN[];         // 8. Left joystick - reverse direction 

//=======================================================================================


//=======================================================================================
// Vehicle -> Ground Station messages 

extern const char 
vehicle_radio_ping_confirm[],   // Ping (heartbeat) confirm 
vehicle_radio_msg_confirm[];    // Command confirm 

//=======================================================================================

#endif   // _COMMANDS_CONFIG_H_ 
