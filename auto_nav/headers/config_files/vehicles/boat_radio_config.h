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
//=======================================================================================


//=======================================================================================
// Macros 

// Thruster command IDs 
#define BOAT_RADIO_ESC_LEFT_MOTOR 0x4C          // "L" character that indicates left motor 
#define BOAT_RADIO_ESC_RIGHT_MOTOR 0x52         // "R" character that indicates right motor 
#define BOAT_RADIO_ESC_FWD_THRUST 0x50          // "P" (plus) - indicates forward thrust 
#define BOAT_RADIO_ESC_REV_THRUST 0x4D          // "M" (minus) - indicates reverse thrust 

//=======================================================================================


//=======================================================================================
// Incoming commands 

#define BOAT_RADIO_NUM_CMDS 9 

extern const char 
// Ground station commands 
boat_radio_cmd_ping[],       // 0. Ping (heartbeat) 
boat_radio_cmd_idle[],       // 1. Idle (standby) state 
boat_radio_cmd_auto[],       // 2. Autonomous state 
boat_radio_cmd_manual[],     // 3. Manual (remote) control state 
boat_radio_cmd_index[],      // 4. Waypoint index set 
boat_radio_cmd_RP[],         // 5. Right thruster - forward thrust 
boat_radio_cmd_RN[],         // 6. Right thruster - reverse thrust 
boat_radio_cmd_LP[],         // 7. Left thruster - forward thrust 
boat_radio_cmd_LN[];         // 8. Left thruster - reverse thrust 

//=======================================================================================


//=======================================================================================
// Outgoing messages 

extern const char 
boat_radio_ping_confirm[],   // Ping (heartbeat) confirm 
boat_radio_msg_confirm[];    // Command confirm 

//=======================================================================================

#endif   // _BOAT_RADIO_CONFIG_H_ 
