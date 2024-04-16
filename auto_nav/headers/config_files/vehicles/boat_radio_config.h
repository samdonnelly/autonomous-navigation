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

extern const std::string 
boat_radio_ping,           // 0. Ping (heartbeat) 
boat_radio_idle,           // 1. Idle (standby) state 
boat_radio_auto,           // 2. Autonomous state 
boat_radio_manual,         // 3. Manual (remote) control state 
boat_radio_index,          // 4. Waypoint index set 
boat_radio_RP,             // 5. Right thruster - forward thrust 
boat_radio_RN,             // 6. Right thruster - reverse thrust 
boat_radio_LP,             // 7. Left thruster - forward thrust 
boat_radio_LN;             // 8. Left thruster - reverse thrust 

//=======================================================================================

#endif   // _BOAT_RADIO_CONFIG_H_ 
