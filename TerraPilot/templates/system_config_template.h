/**
 * @file vehicle_config_template.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle configuration template 
 * 
 * @details This is the template file for configuring the autopilot for your vehicle. 
 *          This file should not be included in the build of your project. Instead a 
 *          copy should be made in the specific project, included in the build and 
 *          modified from there. Values here that are non-zero are the recommended 
 *          starting point. 
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
#define VS_MISSION_TIMEOUT 10      // Max count before mission protocol timeout 
#define VS_MISSION_RESEND 5        // Max tries to resend a mission message 
#define VS_NAV_DEVICE_TIMEOUT 10   // Max count before navigation devices are considered lost 
#define VS_RC_TIMEOUT 10           // Max count before RC comms are considered lost 

// Data sizes 
#define VS_TELEMETRY_BUFF 1000   // Telemetry data buffer size (bytes) 

// The vehicle specific settings below only have an affect when the corresponding vehicle 
// type (VT) is selected. 

// Boat 
#define VS_BOAT_K1 0   // Kinematics 1 - Differential thruster - 2 propellers 
#define VS_BOAT_K2 0   // Kinematics 2 - Rudder - 1 propeller + 1 rudder 

//=======================================================================================

#endif   // _SYSTEM_CONFIG_TEMPLATE_H_ 
