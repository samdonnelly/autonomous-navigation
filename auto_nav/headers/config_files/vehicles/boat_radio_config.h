/**
 * @file boat_radio_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat radio module config interface 
 * 
 * @version 0.1
 * @date 2024-04-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_RADIO_CONFIG_H_ 
#define _BOAT_RADIO_CONFIG_H_ 

//=======================================================================================
// Includes 

#include <string> 

//=======================================================================================


//=======================================================================================
// Radio messages 

#define BOAT_RADIO_NUM_CMDS 9 

// Ground station commands 
extern const std::string 
boat_radio_ping_cmd,       // 0. Ping (heartbeat) 
boat_radio_idle_cmd,       // 1. Idle (standby) state 
boat_radio_auto_cmd,       // 2. Autonomous state 
boat_radio_manual_cmd,     // 3. Manual (remote) control state 
boat_radio_index_cmd,      // 4. Waypoint index set 
boat_radio_RP_cmd,         // 5. Right thruster - forward thrust 
boat_radio_RN_cmd,         // 6. Right thruster - reverse thrust 
boat_radio_LP_cmd,         // 7. Left thruster - forward thrust 
boat_radio_LN_cmd;         // 8. Left thruster - reverse thrust 


// Command response 
extern const std::string 
boat_radio_confirm_res;    // 0. Command confirm 

//=======================================================================================

#endif   // _BOAT_RADIO_CONFIG_H_ 
