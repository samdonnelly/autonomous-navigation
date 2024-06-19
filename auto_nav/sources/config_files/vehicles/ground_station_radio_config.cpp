/**
 * @file ground_station_radio_config.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Ground station radio configuration 
 * 
 * @version 0.1
 * @date 2024-06-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "ground_station_radio_config.h" 

//=======================================================================================


//=======================================================================================
// Incoming commands 

const char 
// User commands 
gs_radio_cmd_manual[] = "mc",          // 0. Manual control mode 
gs_radio_cmd_rf_channel[] = "rf_ch",   // 1. RF channel set 
gs_radio_cmd_rf_power[] = "rf_pwr",    // 2. RF power set 
gs_radio_cmd_rf_dr[] = "rf_dr",        // 3. RF data rate set 
gs_radio_cmd_rf_dp[] = "rf_dp";        // 4. RF data pipe set 

//=======================================================================================


//=======================================================================================
// Outgoing messages 

const char 
gs_radio_confirm[] = "";      // 

//=======================================================================================

