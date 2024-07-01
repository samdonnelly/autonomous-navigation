/**
 * @file vehicle_radio_config.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle radio configuration 
 * 
 * @version 0.1
 * @date 2024-06-21
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle_radio_config.h" 

//=======================================================================================


//=======================================================================================
// Ground Station -> Vehicle commands 

// Ground station commands 
const char 
vehicle_radio_cmd_ping[] = "ping",          // 0. Ping (heartbeat) 
vehicle_radio_cmd_idle[] = "idle",          // 1. Idle (standby) state 
vehicle_radio_cmd_auto[] = "auto",          // 2. Autonomous state 
vehicle_radio_cmd_manual[] = "manual",      // 3. Manual (remote) control state 
vehicle_radio_cmd_index[] = "index",        // 4. Waypoint index set 
vehicle_radio_cmd_RP[] = "RP",              // 5. Right joystick - forward direction 
vehicle_radio_cmd_RN[] = "RN",              // 6. Right joystick - no direction (neutral) 
vehicle_radio_cmd_RM[] = "RM",              // 7. Right joystick - reverse direction 
vehicle_radio_cmd_LP[] = "LP",              // 8. Left joystick - forward direction 
vehicle_radio_cmd_LN[] = "LN",              // 9. Left joystick - no direction (neutral) 
vehicle_radio_cmd_LM[] = "LM";              // 10. Left joystick - reverse direction 

//=======================================================================================


//=======================================================================================
// Vehicle -> Ground Station messages 

const char 
vehicle_radio_ping_confirm[] = "pong",      // Ping (heartbeat) confirm 
vehicle_radio_msg_confirm[] = "confirm";    // Command confirm 

//=======================================================================================
