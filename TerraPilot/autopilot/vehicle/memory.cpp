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
    {"CRUISE_SPEED", &params.cruise_speed, MAV_PARAM_TYPE_REAL32},    // 1 
    {"FRAME_CLASS",  &params.frame_class, MAV_PARAM_TYPE_REAL32},     // 2 
    {"TURN_RADIUS",  &params.turn_radius, MAV_PARAM_TYPE_REAL32},     // 3 
    {"LOIT_TYPE",    &params.loit_type, MAV_PARAM_TYPE_REAL32},       // 4 
    {"LOIT_RADIUS",  &params.loit_radius, MAV_PARAM_TYPE_REAL32}      // 5 
}};

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleMemory::VehicleMemory()
{
    memset((void *)&mission.items, RESET, sizeof(mission.items)); 
    mission.target = HOME_OFFSET; 
    mission.size = HOME_OFFSET; 
    mission.id = RESET; 
    mission.type = MAV_MISSION_TYPE_MISSION; 
}

//=======================================================================================


//=======================================================================================
// Parameters 

// Check if an index is within the parameter size 
bool VehicleMemory::ParameterIndexCheck(uint8_t param_index)
{
    return (param_index < parameters.size()); 
}


// Look up a parameter using the ID/name 
ParamIndex VehicleMemory::ParameterLookUp(const char *param_id)
{
    ParamIndex param_index = RESET; 

    do
    {
        if (!strcmp(param_id, parameters[param_index].name))
        {
            break; 
        }
    } 
    while (++param_index < parameters.size());

    return param_index; 
}


// Set the parameter that matches the ID/name 
ParamIndex VehicleMemory::ParameterSet(
    char *param_id, 
    float &value)
{
    ParamIndex param_index = ParameterLookUp(param_id); 

    if (ParameterIndexCheck(param_index))
    {
        // Use parameters[param_index].type to properly store the value. The stored 
        // type should be used to figure out how to store the value, not the type 
        // received from the GCS. 

        *parameters[param_index].value = value; 
    }

    return param_index; 
}

//=======================================================================================


//=======================================================================================
// Mission 

// Load a stored mission if it exists 
void VehicleMemory::MissionLoad(void)
{
    //==================================================
    // Test 
    
    // Manually setting the home location for now 
    mission.items[HOME_INDEX] = 
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


// Get a mission item 
MissionItem VehicleMemory::MissionItemGet(uint16_t sequence)
{
    MissionItem mission_item; 

    if (sequence < mission.size)
    {
        mission_item = mission.items[sequence]; 
    }
    else 
    {
        mission_item.seq = ~RESET; 
    }

    return mission_item; 
}


// Set a mission item 
void VehicleMemory::MissionItemSet(MissionItem &mission_item)
{
    if (mission_item.seq < MAX_MISSION_SIZE)
    {
        uint16_t index = mission_item.seq + HOME_OFFSET; 
        mission.items[index] = mission_item; 
        mission.items[index].seq = index; 
    }
}


// Set the home location 
void VehicleMemory::MissionHomeSet(MissionItem &mission_item)
{
    mission.items[HOME_INDEX] = mission_item; 
}


// Get the target mission item index 
MissionIndex VehicleMemory::MissionTargetGet(void)
{
    return mission.target; 
}


/**
 * @brief Set the target mission item index 
 * 
 * @param sequence : index of mission to target 
 * @return true/false : target index update success status 
 */
bool VehicleMemory::MissionTargetSet(uint16_t sequence)
{
    if (sequence < mission.size)
    {
        mission.target = sequence; 
        return true; 
    }

    return false; 
}


/**
 * @brief Get the total mission size 
 * 
 * @return MissionSize : mission size including the home location 
 */
MissionSize VehicleMemory::MissionSizeGet(void)
{
    return mission.size; 
}


/**
 * @brief Set the total mission size 
 * 
 * @details Size should be provided as a count, not an index, meaning the size can be up 
 *          to and including the max mission size. The provided size should also not 
 *          count the home location as part of the size. The home location offset is 
 *          added after verifying the size is within range as the home location will be 
 *          used for mission item index 0. Mission Planner will upload missions with 
 *          items and a count that exclude the home location which is why this is done. 
 * 
 * @param size : size of mission (not including the home location) 
 * @return true/false : mission size set success status 
 */
bool VehicleMemory::MissionSizeSet(uint16_t size)
{
    if (size <= MAX_MISSION_SIZE)
    {
        mission.size = size + HOME_OFFSET; 
        return true; 
    }

    return false; 
}


/**
 * @brief Return the current mission type 
 * 
 * @return uint8_t : mission type 
 */
uint8_t VehicleMemory::MissionTypeGet(void)
{
    return mission.type; 
}


/**
 * @brief Set the mission type 
 * 
 * @details This is provided by the GCS when a new mission is uploaded. On successful 
 *          mission upload, the new type can be copied here. 
 * 
 * @param type : mission type 
 */
void VehicleMemory::MissionTypeSet(uint8_t type)
{
    mission.type = type; 
}


/**
 * @brief Return the current mission ID number 
 * 
 * @return uint32_t : mission ID number 
 */
uint32_t VehicleMemory::MissionIDGet(void)
{
    return mission.id; 
}


/**
 * @brief Update and return the mission ID number 
 * 
 * @details The mission ID number simply has to be different than the previous mission 
 *          so the GCS can check which mission a vehicle has without reading the whole 
 *          mission. This system increments the ID whenever there's a mission change 
 *          such as a mission upload or mission clear. 
 * 
 * @return uint32_t : mission ID number 
 */
uint32_t VehicleMemory::MissionIDUpdate(void)
{
    return ++mission.id; 
}


/**
 * @brief Clear the stored mission exluding the home location 
 */
void VehicleMemory::MissionClear(void)
{
    memset((void *)&mission.items[HOME_OFFSET], RESET, MAX_MISSION_SIZE*sizeof(MissionItem)); 
    mission.size = HOME_OFFSET; 
}

//=======================================================================================
