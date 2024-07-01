/**
 * @file vehicle_radio_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle radio configuration interface 
 * 
 * @version 0.1
 * @date 2024-06-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _VEHICLE_RADIO_CONFIG_H_ 
#define _VEHICLE_RADIO_CONFIG_H_ 

//=======================================================================================
// Includes 
//=======================================================================================


//=======================================================================================
// Macros 

// Joystick position IDs 
#define GS_RADIO_LEFT_JOYSTICK 0x4C    // "L" character that indicates left joystick 
#define GS_RADIO_RIGHT_JOYSTICK 0x52   // "R" character that indicates right joystick 
#define GS_RADIO_FWD_DIRECTION 0x50    // "P" (plus) - indicates forward direction 
#define GS_RADIO_REV_DIRECTION 0x4D    // "M" (minus) - indicates reverse direction 
#define GS_RADIO_NEUTRAL 0x4E          // "N" (neutral) - indicates no direction 

//=======================================================================================


//=======================================================================================
// Ground Station -> Vehicle commands 

#define VEHICLE_RADIO_NUM_CMDS 11 

extern const char 
vehicle_radio_cmd_ping[],       // 0. Ping (heartbeat) 
vehicle_radio_cmd_idle[],       // 1. Idle (standby) state 
vehicle_radio_cmd_auto[],       // 2. Autonomous state 
vehicle_radio_cmd_manual[],     // 3. Manual (remote) control state 
vehicle_radio_cmd_index[],      // 4. Waypoint index set 
vehicle_radio_cmd_RP[],         // 5. Right joystick - forward direction 
vehicle_radio_cmd_RN[],         // 6. Right joystick - no direction (neutral)
vehicle_radio_cmd_RM[],         // 7. Right joystick - reverse direction 
vehicle_radio_cmd_LP[],         // 8. Left joystick - forward direction 
vehicle_radio_cmd_LN[],         // 9. Left joystick - no direction (neutral) 
vehicle_radio_cmd_LM[];         // 10. Left joystick - reverse direction 

//=======================================================================================


//=======================================================================================
// Vehicle -> Ground Station messages 

extern const char 
vehicle_radio_ping_confirm[],   // Ping (heartbeat) confirm 
vehicle_radio_msg_confirm[];    // Command confirm 

//=======================================================================================

#endif   // _VEHICLE_RADIO_CONFIG_H_ 
