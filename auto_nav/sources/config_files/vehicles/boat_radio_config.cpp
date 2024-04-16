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

const std::string 
boat_radio_ping = "ping",       // 0. Ping (heartbeat) 
boat_radio_idle = "idle",       // 1. Idle (standby) state 
boat_radio_auto = "auto",       // 2. Autonomous state 
boat_radio_manual = "manual",   // 3. Manual (remote) control state 
boat_radio_index = "index",     // 4. Waypoint index set 
boat_radio_RP = "RP",           // 5. Right thruster - forward thrust 
boat_radio_RN = "RN",           // 6. Right thruster - reverse thrust 
boat_radio_LP = "LP",           // 7. Left thruster - forward thrust 
boat_radio_LN = "LN";           // 8. Left thruster - reverse thrust 

//=======================================================================================
