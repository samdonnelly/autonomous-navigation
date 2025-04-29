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
// Macros 

#define HOME_RADIUS 3   // meters 

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
    {"CRUISE_SPEED", &params.cruise_speed, MAV_PARAM_TYPE_REAL32},   // 1 
    {"FRAME_CLASS",  &params.frame_class, MAV_PARAM_TYPE_REAL32},    // 2 
    {"TURN_RADIUS",  &params.turn_radius, MAV_PARAM_TYPE_REAL32},    // 3 
    {"LOIT_TYPE",    &params.loit_type, MAV_PARAM_TYPE_REAL32},      // 4 
    {"LOIT_RADIUS",  &params.loit_radius, MAV_PARAM_TYPE_REAL32}     // 5 
}};

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleMemory::VehicleMemory()
{
    memset((void *)&mission.items, RESET, sizeof(mission.items)); 
    mission.target = HOME_INDEX; 
    mission.size = RESET; 
    mission.id = RESET; 
    mission.type = MAV_MISSION_TYPE_MISSION; 

    // Default home item info. May change with Mission Planner uploads. 
    mission.items[HOME_INDEX] = 
    {
        .param1 = RESET,                              // Hold - home location will hold indefinitely 
        .param2 = HOME_RADIUS,                        // Waypoint acceptance radius 
        .param3 = RESET,                              // Waypoint pass radius - pass through 
        .param4 = RESET,                              // Yaw angle at waypoint 
        .x = RESET,                                   // Home latitude - set by system or GCS 
        .y = RESET,                                   // Home longitude - set by system or GCS 
        .z = RESET,                                   // Home altitude - set by system or GCS 
        .seq = HOME_INDEX,                            // Waypoint ID - home location index 
        .command = MAV_CMD_NAV_WAYPOINT,              // Command - treat as waypoint 
        .target_system = VS_SYSTEM_ID,                // System ID - same as received mission item 
        .target_component = MAV_COMP_ID_AUTOPILOT1,   // Component ID - same as received mission item 
        .frame = MAV_FRAME_GLOBAL,                    // Coordinate frame 
        .current = true,                              // Current target item - true to start 
        .autocontinue = false,                        // Autocontinue to next waypoint - false 
        .mission_type = MAV_MISSION_TYPE_MISSION      // Mission type - generic waypoint 
    };

    status.flags = RESET; 
}

//=======================================================================================


//=======================================================================================
// Parameters 

/**
 * @brief Check if an index is within the parameter size 
 * 
 * @param param_index : index of parameter within 'parameters' 
 * @return true/false : status of the check - true if within range 
 */
bool VehicleMemory::ParameterIndexCheck(uint8_t param_index)
{
    return (param_index < parameters.size()); 
}


/**
 * @brief Look up a parameter using the ID/name 
 * 
 * @param param_id : name/key/ID of parameter used to search for a parameter 
 * @return ParamIndex : index of specified parameter - index == parameters.size() if invalid 
 */
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


/**
 * @brief Set the parameter that matches the ID/name 
 * 
 * @param param_id : name/key/ID of parameter used to search for a parameter 
 * @param value : number to set parameter to if ID is valid 
 * @return ParamIndex : index of specified parameter - index == parameters.size() if invalid 
 */
ParamIndex VehicleMemory::ParameterSet(
    const char *param_id, 
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

/**
 * @brief Load a stored mission if it exists 
 */
void VehicleMemory::MissionLoad(void)
{
    // Do not set the home location flag. Retrieving the home position from memory 
    // (mission item 0) is not the same as explicitly setting the home location. 
    // The home location flag is also not cleared because we don't want the system to 
    // change the home position in the event of a reset. The home position flag will be 
    // cleared on startup which will prompt the system to obtain an updated home 
    // position for the current session. 

    //==================================================
    // Test 
    
    // Force the home location to be set for testing in the absence of GPS. 
    MissionHomeLocationSet(506132550, -1151204500, 0); 
    
    //==================================================
}


/**
 * @brief Get a mission item 
 * 
 * @param sequence : index of mission item to get 
 * @return MissionItem : mission item based on sequence number 
 */
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


/**
 * @brief Set a mission item 
 * 
 * @details When uploading a mission to the vehicle, Mission Planner will send the home 
 *          location it has stored locally as the first item (index 0) so no index 
 *          correction is needed. If the home location is seen then the home location 
 *          flag is set so the vehicle won't set it's own home position during the 
 *          startup sequence. 
 * 
 * @param mission_item : item and sequence to set 
 */
void VehicleMemory::MissionItemSet(MissionItem &mission_item)
{
    if (mission_item.seq < mission.items.size())
    {
        mission.items[mission_item.seq] = mission_item; 

        if (mission_item.seq == HOME_INDEX)
        {
            status.home_location = FLAG_SET; 
        }
    }
}


/**
 * @brief Set the home location 
 * 
 * @details This provides a means to update the location only for the home position since 
 *          the home position is stored as a mission item at index 0. If the mission size 
 *          is zero it means the home position hasn't been set yet so the mission size is 
 *          updated to reflect this. 
 * 
 * @param lat : home latitude 
 * @param lon : home longitude 
 * @param alt : home altitude 
 */
void VehicleMemory::MissionHomeLocationSet(
    int32_t lat, 
    int32_t lon, 
    float alt)
{
    mission.items[HOME_INDEX].x = lat; 
    mission.items[HOME_INDEX].y = lon; 
    mission.items[HOME_INDEX].z = alt; 

    status.home_location = FLAG_SET; 

    // If the mission size is greater than 0 it means there are mission items that we 
    // can't ignore. 
    if (mission.size == RESET)
    {
        mission.size = HOME_OFFSET; 
    }
}


/**
 * @brief Indicates if the home position has been set or not 
 * 
 * @return true/false : home location set status 
 */
bool VehicleMemory::MissionHomeLocationStatus(void)
{
    return status.home_location; 
}


/**
 * @brief Get the target mission item index 
 * 
 * @return MissionIndex : index of current target mission item 
 */
MissionIndex VehicleMemory::MissionTargetGet(void)
{
    return mission.target; 
}


/**
 * @brief Set the target mission item index 
 * 
 * @details The 'current' status is also updated for the previous and new waypoints. 
 * 
 * @param sequence : index of mission to target 
 * @return true/false : target index update success status 
 */
bool VehicleMemory::MissionTargetSet(uint16_t sequence)
{
    if (sequence < mission.size)
    {
        mission.items[mission.target].current = false; 
        mission.items[sequence].current = true; 
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
 *          to and including the max mission size. Mission Planner counts the home 
 *          location as the first mission item. If the size is larger than the available 
 *          mission storage space then it defaults to the max size. 
 * 
 * @param size : size of mission (not including the home location) 
 */
void VehicleMemory::MissionSizeSet(uint16_t size)
{
    mission.size = (size <= mission.items.size()) ? size : mission.items.size(); 
}


/**
 * @brief Return the current mission type 
 * 
 * @details Types are based on MAV_MISSION_TYPE but this datatype is not explicitly used. 
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
 *          mission upload, the new type can be copied here. Types are based on 
 *          MAV_MISSION_TYPE but this datatype is not explicitly used. 
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
