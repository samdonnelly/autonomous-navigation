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

// The following parameters are available across the vehicle system but cannot be 
// modified outside of this file. Only parameter values can be changed and it's done 
// using the ParameterValues struct. 

const std::array<VehicleMemory::ParamInfo, NUM_PARAMETERS> parameters = 
{{
    {"CRUISE_SPEED", &params.cruise_speed},    // 1 
    {"FRAME_CLASS",  &params.frame_class},     // 2 
    {"TURN_RADIUS",  &params.turn_radius},     // 3 
    {"LOIT_TYPE",    &params.loit_type},       // 4 
    {"LOIT_RADIUS",  &params.loit_radius}      // 5 
}};

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleMemory::VehicleMemory()
    : param_index(RESET), 
      param_value_type(MAV_PARAM_TYPE_REAL32), 
      mission_size(RESET), 
      mission_id(RESET), 
      mission_type(MAV_MISSION_TYPE_ALL), 
      mission_index(RESET) 
{
    memset((void *)mission, RESET, sizeof(mission)); 
}

//=======================================================================================


//=======================================================================================
// Parameters 

void VehicleMemory::ParameterLookUp(char *param_id)
{
    for (uint8_t i = RESET; i < parameters.size(); i++)
    {
        if (!strcmp(param_id, parameters[i].name))
        {
            param_index = i; 
            return; 
        }
    }

    param_index = parameters.size(); 
}


void VehicleMemory::ParameterSet(
    char *param_id, 
    float &value)
{
    ParameterLookUp(param_id); 

    if (param_index < parameters.size())
    {
        *parameters[param_index].value = value; 
    }
}

//=======================================================================================


//=======================================================================================
// Mission 

void VehicleMemory::MissionLoad(void)
{
    //==================================================
    // Test 

    mission_size = 1; 
    
    mission[0] = 
    {
        .param1 = 0, 
        .param2 = 10, 
        .param3 = 10, 
        .param4 = 0, 
        .x = 506132550, 
        .y = -1151204500, 
        .z = 0, 
        .seq = 0, 
        .command = MAV_CMD_NAV_WAYPOINT, 
        .target_system = VS_SYSTEM_ID_GCS, 
        .target_component = MAV_COMP_ID_MISSIONPLANNER, 
        .frame = MAV_FRAME_GLOBAL, 
        .current = 0, 
        .autocontinue = 0, 
        .mission_type = MAV_MISSION_TYPE_MISSION 
    };
    
    //==================================================
}

//=======================================================================================
