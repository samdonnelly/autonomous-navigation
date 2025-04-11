/**
 * @file vehicle_config_template.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle configuration template 
 * 
 * @details This is the template file for configuring the autopilot for your vehicle. 
 *          The values here will not be read and should not be altered unless adding 
 *          or removing a configuration. The contents of this file should be copied 
 *          and added to a "system_config.h" file in the same folder as this file. 
 *          "system_config.h" is not tracked but is required and will be read to 
 *          understand how the user wants their vehicle configured. The user should 
 *          change the below values from 0 to 1 in "system_config.h" if they want 
 *          something enabled. Note that every system is different but some items 
 *          such as telemetry and GPS are necessary for most autonomous vehicle 
 *          navigation. 
 * 
 * @version 0.1
 * @date 2025-02-24
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _SYSTEM_CONFIG_TEMPLATE_H_ 
#define _SYSTEM_CONFIG_TEMPLATE_H_ 

//=======================================================================================
// Vehicle Type (VT) 

#define VT_BOAT 0 

//=======================================================================================


//=======================================================================================
// Vehicle Hardware (VH) 

#define VH_RECEIVER 0 
#define VH_TELEMETRY 0 
#define VH_GPS 0 
#define VH_COMPASS 0 
#define VH_IMU 0 
#define VH_SD_CARD 0 
#define VH_ESC 0 
#define VH_ADC 0 
#define VH_LED 0 

//=======================================================================================


//=======================================================================================
// Vehicle Settings (VS) 

// Telemetry ID 
#define VS_SYSTEM_ID 1 
#define VS_SYSTEM_ID_GCS 255 

// Timing 
#define VS_HEARTBEAT_TIMEOUT 100   // Max count before heartbeat timeout 
#define VS_MISSION_TIMEOUT 5       // Max count before mission protocol timeout 
#define VS_MISSION_RESEND 5        // Max tries to resend a mission message 
#define VS_ACK_RESEND 3            // Number of times to resend an acknowledgment 

// Data sizes 
#define VS_TELEMETRY_BUFF 1000   // Telemetry data buffer size (bytes) 

// The vehicle specific settings below only have an affect when the corresponding vehicle 
// type (VT) is selected. 

// Boat 
#define VS_BOAT_K1 0   // Kinematics 1 - Differential thruster - 2 propellers 
#define VS_BOAT_K2 0   // Kinematics 2 - Rudder - 1 propeller + 1 rudder 

//=======================================================================================

#endif   // _SYSTEM_CONFIG_TEMPLATE_H_ 
