/**
 * @file gs_interface_config.h
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

#ifndef _GS_INTERFACE_CONFIG_H_ 
#define _GS_INTERFACE_CONFIG_H_ 

//=======================================================================================
// Includes 
//=======================================================================================


//=======================================================================================
// User -> Ground Station commands 

#define GS_NUM_CMDS 6 

extern const char 
gs_cmd_manual[],        // 0. Manual control mode 
gs_cmd_rf_channel[],    // 1. RF channel set 
gs_cmd_rf_power[],      // 2. RF power set 
gs_cmd_rf_dr[],         // 3. RF data rate set 
gs_cmd_rf_dp[],         // 4. RF data pipe set 
gs_cmd_update[];        // 5. Update serial terminal output 

// Secondary commands 
extern const char 
gs_sub_cmd_on[],        // Enable 
gs_sub_cmd_off[];       // Disable 

//=======================================================================================


//=======================================================================================
// Ground station user interface 

extern const char 
gs_ui_cmd_prompt[],        // User command input prompt 
gs_ui_last_input[],        // Last input by the user 
gs_ui_radio_connect[],     // Radio connection status 
gs_ui_vehicle_msg[],       // Last message received from a vehicle 
gs_ui_cmd_status[],        // Ground station command status 
gs_ui_channel_set[],       // RF module frequency channel 
gs_ui_dr_set[],            // RF module data rate setting 
gs_ui_pwr_set[],           // RF module power output setting 
gs_ui_dp_set[];            // RF module data pipe 

// Command responses 
extern const char 
gs_ui_status_success[],    // Command successfully executed 
gs_ui_status_fail[],       // Command failed 
gs_ui_status_invalid[],    // Invalid command argument 
gs_ui_status_cmd_sent[];   // Command sent via radio to a vehicle 

//=======================================================================================

#endif   // _GS_INTERFACE_CONFIG_H_ 
