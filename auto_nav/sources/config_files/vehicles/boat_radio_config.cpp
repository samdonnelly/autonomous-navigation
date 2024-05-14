/**
 * @file boat_radio_config.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat radio module config 
 * 
 * @version 0.1
 * @date 2024-04-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat_radio_config.h" 

//=======================================================================================


//=======================================================================================
// Radio messages 

// Ground station commands 
const std::string 
boat_radio_ping_cmd = "ping",          // 0. Ping (heartbeat) 
boat_radio_idle_cmd = "idle",          // 1. Idle (standby) state 
boat_radio_auto_cmd = "auto",          // 2. Autonomous state 
boat_radio_manual_cmd = "manual",      // 3. Manual (remote) control state 
boat_radio_index_cmd = "index",        // 4. Waypoint index set 
boat_radio_RP_cmd = "RP",              // 5. Right thruster - forward thrust 
boat_radio_RN_cmd = "RN",              // 6. Right thruster - reverse thrust 
boat_radio_LP_cmd = "LP",              // 7. Left thruster - forward thrust 
boat_radio_LN_cmd = "LN";              // 8. Left thruster - reverse thrust 


// Command response 
extern const std::string 
boat_radio_confirm_res = "confirm";    // 0. Command confirm 

//=======================================================================================
