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

#include "memory.h" 

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

const VehicleMemory::ParamInfo parameters[] = 
{
    {"CRUISE_SPEED", params.cruise_speed}, 
    {"FRAME_CLASS",  params.frame_class}, 
    {"TURN_RADIUS",  params.turn_radius}, 
    {"LOIT_TYPE",    params.loit_type}, 
    {"LOIT_RADIUS",  params.loit_radius} 
}; 

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleMemory::VehicleMemory()
    : param_value_type(MAV_PARAM_TYPE_REAL32)
{
    num_params = sizeof(parameters) / sizeof(*parameters); 
}

//=======================================================================================
