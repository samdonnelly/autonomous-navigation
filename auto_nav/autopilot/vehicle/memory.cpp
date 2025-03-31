/**
 * @file memory.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle memory 
 * 
 * @version 0.1
 * @date 2025-03-17
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Notes 

// Mission storage 
// - Two mission files, one to store the current mission the another to record a new 
//   mission when it arrives. If a new mission upload is accepted then the system 
//   switches to using the file with the new mission. An index that indicates which file 
//   to use is stored in another file so that it can be kept track of between power 
//   cycles. 
// - A settings files which stores: 
//   - The aforementioned mission file index 
//   - Mission ID (opaque ID) - increments on new mission upload 
//   - Home location (so it can't be wiped by a mission upload) 
//     - On this note, make sure to write this as the first item in a mission file. 

//=======================================================================================


//=======================================================================================
// Structs 

struct ParameterValues 
{
    float 
    cruise_speed,   // Target cruise speed in auto mode (m/s) 
    frame_class,    // Frame class: Boat 
    turn_radius,    // Turn radius of vehicle (meters) 
    loit_type,      // Loiter type: Forward or reverse to target point 
    loit_radius;    // Loiter radius (meters) 
}; 

static ParameterValues params; 

//=======================================================================================


//=======================================================================================
// Parameters 

const std::array<VehicleMemory::ParamInfo, NUM_PARAMETERS> parameters = 
{{
    {"CRUISE_SPEED", params.cruise_speed},    // 1 
    {"FRAME_CLASS",  params.frame_class},     // 2 
    {"TURN_RADIUS",  params.turn_radius},     // 3 
    {"LOIT_TYPE",    params.loit_type},       // 4 
    {"LOIT_RADIUS",  params.loit_radius}      // 5 
}}; 

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleMemory::VehicleMemory()
    : param_index(RESET), 
      param_value_type(MAV_PARAM_TYPE_REAL32), 
      mission_size(RESET), 
      mission_id(RESET), 
      mission_index(RESET) 
{
    num_params = sizeof(parameters) / sizeof(parameters[0]); 
}

//=======================================================================================
