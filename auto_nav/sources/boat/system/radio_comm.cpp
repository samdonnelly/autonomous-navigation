/**
 * @file radio_comm.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB radio control 
 * 
 * @version 0.1
 * @date 2024-03-14
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "radio_comm.h" 
#include "ab_interface.h" 
#include "led_control.h" 
#include "gps_coordinates.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Navigation 
#define AB_GPS_INDEX_CNT 3           // Successive index command count needed to update 

//=======================================================================================


//=======================================================================================
// Prototypes 

// /**
//  * @brief Idle command 
//  * 
//  * @details Run when the ground station sends an "idle" command while in an applicable 
//  *          state. This will trigger idle mode and turn the thrusters off. See the 
//  *          "cmd_table" for states in which this command is valid. 
//  * 
//  * @param idle_cmd_value : generic idle command argument 
//  */
// void ab_idle_cmd(uint8_t idle_cmd_value); 


// /**
//  * @brief Manual control mode command 
//  * 
//  * @details Run when the ground station sends a "manual" command while in an applicable 
//  *          state. This will trigger manual control mode. See the "cmd_table" for states 
//  *          in which this command is valid. 
//  * 
//  * @param manual_cmd_value : generic manual mode command argument 
//  */
// void ab_manual_cmd(uint8_t manual_cmd_value); 


// /**
//  * @brief Autonomous mode command 
//  * 
//  * @details Run when the ground station sends an "auto" command while in an applicable 
//  *          state. Puts the system in autonomous mode. See the "cmd_table" for states in 
//  *          which this command is valid. 
//  * 
//  * @param auto_cmd_value : generic autonomous mode command argument 
//  */
// void ab_auto_cmd(uint8_t auto_cmd_value); 


// /**
//  * @brief Index update command 
//  * 
//  * @details Run when the ground station sends an "index" command while in an applicable 
//  *          state. The payload of the index command ("index <payload>" - passed as the 
//  *          'index_cmd_value' argument) contains the index used to set the target 
//  *          location within the pre-defined waypoint mission. This function will verify 
//  *          the index value and update the target location if the index is valid. See 
//  *          the "cmd_table" for states in which this command is valid. 
//  * 
//  * @param index_cmd_value : generic index update command argument 
//  */
// void ab_index_cmd(uint8_t index_cmd_value); 


// /**
//  * @brief Manual throttle command 
//  * 
//  * @details Run when the ground station sends an "RP", "RN", "LP", or "LN" command while 
//  *          in an applicable state. These indicate right or left thruster as well as 
//  *          positive (forward) and negative (reverse) thrust. Each command will be 
//  *          followed by a payload (ex. "RP <payload>") that indicates the thruster 
//  *          command (%), however this payload is used in the "manual" state function and 
//  *          not here. This function will indicate when a manual control message has been 
//  *          received and will reset the heartbeat timeout counter. See the "cmd_table" 
//  *          for states in which this command is valid. 
//  * 
//  * @param throttle_cmd_value : generic manual throttle command argument 
//  */
// void ab_throttle_cmd(uint8_t throttle_cmd_value); 


// /**
//  * @brief Heartbeat command 
//  * 
//  * @details Run when the ground station sends an "ping" command while in an applicable 
//  *          state. This function resets the heartbeat timeout counter. The ground 
//  *          station will periodically send a "ping" and the boat uses this to know if 
//  *          it still has radio communication with the ground station. See the "cmd_table"
//  *          for states in which this command is valid. 
//  * 
//  * @param hb_cmd_value : generic heartbeat command argument 
//  */
// void ab_hb_cmd(uint8_t hb_cmd_value); 

//=======================================================================================


//=======================================================================================
// Radio functions 

// Check for and handle commands 


// Heartbeat 


// // Parse the ground station command into an ID and value 
// uint8_t ab_parse_cmd(uint8_t *command_buffer)
// {
//     uint8_t id_flag = SET_BIT; 
//     uint8_t id_index = CLEAR; 
//     uint8_t data = CLEAR; 
//     uint8_t cmd_value[AB_MAX_CMD_SIZE]; 
//     uint8_t value_size = CLEAR; 

//     // Initialize data 
//     memset((void *)ab_data.cmd_id, CLEAR, sizeof(ab_data.cmd_id)); 
//     ab_data.cmd_value = CLEAR; 
//     memset((void *)cmd_value, CLEAR, sizeof(cmd_value)); 

//     // Parse the command into an ID and value 
//     for (uint8_t i = CLEAR; command_buffer[i] != NULL_CHAR; i++)
//     {
//         data = command_buffer[i]; 

//         if (id_flag)
//         {
//             // cmd ID parsing 

//             id_index = i; 

//             // Check that the command byte is within range 
//             if ((data >= A_LO_CHAR && data <= Z_LO_CHAR) || 
//                 (data >= A_UP_CHAR && data <= Z_UP_CHAR))
//             {
//                 // Valid character byte seen 
//                 ab_data.cmd_id[i] = data; 
//             }
//             else if (data >= ZERO_CHAR && data <= NINE_CHAR)
//             {
//                 // Valid digit character byte seen 
//                 id_flag = CLEAR_BIT; 
//                 ab_data.cmd_id[i] = NULL_CHAR; 
//                 cmd_value[i-id_index] = data; 
//                 value_size++; 
//             }
//             else 
//             {
//                 // Valid data not seen 
//                 return FALSE; 
//             }
//         }
//         else 
//         {
//             // cmd value parsing 

//             if (data >= ZERO_CHAR && data <= NINE_CHAR)
//             {
//                 // Valid digit character byte seen 
//                 cmd_value[i-id_index] = data; 
//                 value_size++; 
//             }
//             else 
//             {
//                 // Valid data not seen 
//                 return FALSE; 
//             }
//         }
//     }

//     // Calculate the cmd value 
//     for (uint8_t i = CLEAR; i < value_size; i++)
//     {
//         ab_data.cmd_value += (uint8_t)char_to_int(cmd_value[i], value_size-i-1); 
//     }

//     return TRUE; 
// }


// // Idle command 
// void ab_idle_cmd(uint8_t idle_cmd_value)
// {
//     ab_data.idle = SET_BIT; 
//     ab_data.state_entry = SET_BIT; 
//     ab_data.manual = CLEAR_BIT; 
//     ab_data.autonomous = CLEAR_BIT; 
//     ab_data.nav_timer.time_start = SET_BIT; 

//     // Set the throttle to zero to stop the thrusters 
//     ab_data.right_thruster = AB_NO_THRUST; 
//     ab_data.left_thruster = AB_NO_THRUST; 
//     esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
//     esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 

//     // Make sure the LEDs are off and reset the strobe timer 
//     ab_led_strobe_off(); 
// }


// // Manual control mode command 
// void ab_manual_cmd(uint8_t manual_cmd_value)
// {
//     ab_data.manual = SET_BIT; 
//     ab_data.state_entry = SET_BIT; 
//     ab_data.idle = CLEAR_BIT; 

//     // Make sure the LEDs are off and reset the strobe timer 
//     ab_led_strobe_off(); 
// }


// // Autonomous mode command 
// void ab_auto_cmd(uint8_t auto_cmd_value)
// {
//     ab_data.autonomous = SET_BIT; 
//     ab_data.state_entry = SET_BIT; 
//     ab_data.idle = CLEAR_BIT; 

//     // Make sure the LEDs are off and reset the strobe timer 
//     ab_led_strobe_off(); 
// }


// // Index update command 
// void ab_index_cmd(uint8_t index_cmd_value)
// {
//     static uint8_t index_check = CLEAR; 
//     static uint8_t index_last = CLEAR; 

//     // Compare the previous index command to the new index command. The radio messages 
//     // between the ground station and boat are poor meaning a complete and correct 
//     // message often does not get transmitted and received successfully. This can lead 
//     // to the index not being updated to the desired value and therefore the boat moving 
//     // to a target it's not supposed to. To combat this, the index has to been seen 
//     // successively at least "AB_GPS_INDEX_CNT" times before the index will be updated. 
//     if (index_cmd_value != index_last)
//     {
//         index_last = index_cmd_value; 
//         index_check = SET_BIT; 
//     }
//     else 
//     {
//         index_check++; 
//     }

//     // Check that the index is within bounds and seen the last AB_GPS_INDEX_CNT times before 
//     // updating the index (filters noise). 
//     if ((index_cmd_value < NUM_GPS_WAYPOINTS) && (index_check >= AB_GPS_INDEX_CNT))
//     {
//         ab_data.waypoint_index = index_cmd_value; 
//         ab_data.target.lat = gps_waypoints[ab_data.waypoint_index].lat; 
//         ab_data.target.lon = gps_waypoints[ab_data.waypoint_index].lon; 
//         index_check = CLEAR; 
//     }
// }


// // Manual throttle command 
// void ab_throttle_cmd(uint8_t throttle_cmd_value)
// {
//     ab_data.mc_data = SET_BIT; 
//     ab_data.connect = SET_BIT; 
//     ab_data.hb_timeout = CLEAR; 
// }


// // Heartbeat command 
// void ab_hb_cmd(uint8_t hb_cmd_value)
// {
//     ab_data.connect = SET_BIT; 
//     ab_data.hb_timeout = CLEAR; 
// }

//=======================================================================================
