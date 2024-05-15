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
// Incoming commands 

const std::string 
// Ground station commands 
boat_radio_cmd_ping = "ping",          // 0. Ping (heartbeat) 
boat_radio_cmd_idle = "idle",          // 1. Idle (standby) state 
boat_radio_cmd_auto = "auto",          // 2. Autonomous state 
boat_radio_cmd_manual = "manual",      // 3. Manual (remote) control state 
boat_radio_cmd_index = "index",        // 4. Waypoint index set 
boat_radio_cmd_RP = "RP",              // 5. Right thruster - forward thrust 
boat_radio_cmd_RN = "RN",              // 6. Right thruster - reverse thrust 
boat_radio_cmd_LP = "LP",              // 7. Left thruster - forward thrust 
boat_radio_cmd_LN = "LN";              // 8. Left thruster - reverse thrust 

//=======================================================================================


//=======================================================================================
// Outgoing messages 

extern const std::string 
boat_radio_msg_confirm = "confirm";    // Command confirm 

//=======================================================================================
