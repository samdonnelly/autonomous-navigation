/**
 * @file commands_config.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief System commands configuration 
 * 
 * @version 0.1
 * @date 2024-06-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "commands_config.h" 

//=======================================================================================


//=======================================================================================
// User -> Ground Station commands 

const char 
gs_cmd_manual[] = "mc",          // 0. Manual control mode 
gs_cmd_rf_channel[] = "rf_ch",   // 1. RF channel set 
gs_cmd_rf_power[] = "rf_pwr",    // 2. RF power set 
gs_cmd_rf_dr[] = "rf_dr",        // 3. RF data rate set 
gs_cmd_rf_dp[] = "rf_dp",        // 4. RF data pipe set 
gs_cmd_update[] = "update";      // 5. Update serial terminal output (user interface) 

// Secondary commands 
extern const char 
gs_sub_cmd_on[] = "on",          // Enable 
gs_sub_cmd_off[] = "off";        // Disable 

//=======================================================================================


//=======================================================================================
// Ground Station -> Vehicle commands 

// Ground station commands 
const char 
vehicle_radio_cmd_ping[] = "ping",          // 0. Ping (heartbeat) 
vehicle_radio_cmd_idle[] = "idle",          // 1. Idle (standby) state 
vehicle_radio_cmd_auto[] = "auto",          // 2. Autonomous state 
vehicle_radio_cmd_manual[] = "manual",      // 3. Manual (remote) control state 
vehicle_radio_cmd_index[] = "index",        // 4. Waypoint index set 
vehicle_radio_cmd_RP[] = "RP",              // 5. Right joystick - forward direction 
vehicle_radio_cmd_RN[] = "RN",              // 6. Right joystick - reverse direction 
vehicle_radio_cmd_LP[] = "LP",              // 7. Left joystick - forward direction 
vehicle_radio_cmd_LN[] = "LN";              // 8. Left joystick - reverse direction 

//=======================================================================================


//=======================================================================================
// Vehicle -> Ground Station messages 

const char 
vehicle_radio_ping_confirm[] = "pong",      // Ping (heartbeat) confirm 
vehicle_radio_msg_confirm[] = "confirm";    // Command confirm 

//=======================================================================================

