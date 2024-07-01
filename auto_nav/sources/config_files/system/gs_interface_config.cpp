/**
 * @file gs_interface_config.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station interface configuration 
 * 
 * @version 0.1
 * @date 2024-06-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "gs_interface_config.h" 

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

// Secondary commands (sub-commands) 
extern const char 
gs_sub_cmd_on[] = "on",          // Enable 
gs_sub_cmd_off[] = "off";        // Disable 

//=======================================================================================


//=======================================================================================
// Ground station user interface 

const char 
gs_ui_cmd_prompt[] = "\r>>> \033[K",              // User command input prompt 
gs_ui_last_input[] = "Last input: %s",            // Last input by the user 
gs_ui_radio_connect[] = "Radio Connection: %u",   // Radio connection status 
gs_ui_vehicle_msg[] = "Vehicle Message: %s",      // Last message received from a vehicle 
gs_ui_cmd_status[] = "Cmd Status: %s",            // Ground station command status 
gs_ui_channel_set[] = "Channel: %u",              // RF module frequency channel 
gs_ui_dr_set[] = "Date Rate: %u",                 // RF module data rate setting 
gs_ui_pwr_set[] = "Power Output: %u",             // RF module power output setting 
gs_ui_dp_set[] = "Data Pipe: %u";                 // RF module data pipe 

// Command responses 
const char 
gs_ui_status_success[] = "Success",               // Command successfully executed 
gs_ui_status_fail[] = "Failed",                   // Command failed 
gs_ui_status_invalid[] = "Invalid arg",           // Invalid command argument 
gs_ui_status_cmd_sent[] = "Command sent";         // Command sent via radio to a vehicle 

//=======================================================================================
