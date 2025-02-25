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
 *          and added to a "vehicle_config.h" file in the same folder as this file. 
 *          "vehicle_config.h" is not tracked but is required and will be read to 
 *          understand how the user wants their vehicle configured. The user should 
 *          change the below values from 0 to 1 in "vehicle_config.h" if they want 
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

#ifndef _VEHICLE_CONFIG_TEMPLATE_H_ 
#define _VEHICLE_CONFIG_TEMPLATE_H_ 

//=======================================================================================
// Vehicle Type 

#define VT_BOAT 0 
#define VT_PLANE 0 

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
// Settings 
//=======================================================================================

#endif   // _VEHICLE_CONFIG_TEMPLATE_H_ 
