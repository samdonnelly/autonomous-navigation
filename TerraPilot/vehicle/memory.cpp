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
// Parameters 

class Parameters 
{
public:   // public types 

    enum ParameterIndex : uint8_t 
    {
        // Accelerometer 
        ACCEL_SX,       // Uncertainty/variance along the X axis (g's) 
        ACCEL_SY,       // Uncertainty/variance along the Y axis (g's) 
        ACCEL_SZ,       // Uncertainty/variance along the Z axis (g's) 
        // Autonomous modes 
        AUTO_MAX_PWM,   // Max PWM output of motor(s) in autonomous modes 
        // Compass 
        COMPASS_TN,     // True North offset (magnetic declination) 
        COMPASS_HIX,    // Hard iron offset on the X axis (milligauss) 
        COMPASS_HIY,    // Hard iron offset on the Y axis (milligauss) 
        COMPASS_HIZ,    // Hard iron offset on the Z axis (milligauss) 
        COMPASS_SIDX,   // Soft iron diagonal X axis component 
        COMPASS_SIDY,   // Soft iron diagonal Y axis component 
        COMPASS_SIDZ,   // Soft iron diagonal Z axis component 
        COMPASS_SIOX,   // Soft iron off-diagonal X axis component 
        COMPASS_SIOY,   // Soft iron off-diagonal Y axis component 
        COMPASS_SIOZ,   // Soft iron off-diagonal Z axis component 
        // Waypoints 
        WP_RADIUS,      // Waypoint radius 
        // Calculations 
        MADGWICK_B      // Madgwick filter 
    };

    struct ParameterValue 
    {
        float 
        // Accelerometer 
        accel_sx,       // Uncertainty/variance along the X axis (g's) 
        accel_sy,       // Uncertainty/variance along the Y axis (g's) 
        accel_sz,       // Uncertainty/variance along the Z axis (g's) 
        // Autonomous modes 
        auto_max_pwm,   // Max PWM output of motor(s) in autonomous modes 
        // Compass 
        compass_tn,     // True North offset (magnetic declination) (degrees) 
        compass_hix,    // Hard iron offset on the X axis (milligauss) 
        compass_hiy,    // Hard iron offset on the Y axis (milligauss) 
        compass_hiz,    // Hard iron offset on the Z axis (milligauss) 
        compass_sidx,   // Soft iron diagonal X axis component 
        compass_sidy,   // Soft iron diagonal Y axis component 
        compass_sidz,   // Soft iron diagonal Z axis component 
        compass_siox,   // Soft iron off-diagonal X axis component 
        compass_sioy,   // Soft iron off-diagonal Y axis component 
        compass_sioz,   // Soft iron off-diagonal Z axis component 
        // Waypoints 
        wp_radius,      // Waypoint radius (meters) 
        // Calculations 
        madgwick_b;     // Madgwick filter 
    }
    values;
}; 

static Parameters params; 


// The following parameters are available across the vehicle system but cannot be 
// modified outside of this file. Only parameter values can be changed and it's done 
// using the ParameterValues struct. 

const std::array<VehicleMemory::ParamInfo, num_parameters> parameters = 
{{
    // Accelerometer 
    {"ACCEL_SX",     &params.values.accel_sx,     MAV_PARAM_TYPE_REAL32, params.ACCEL_SX},       // 1 
    {"ACCEL_SY",     &params.values.accel_sy,     MAV_PARAM_TYPE_REAL32, params.ACCEL_SY},       // 2 
    {"ACCEL_SZ",     &params.values.accel_sz,     MAV_PARAM_TYPE_REAL32, params.ACCEL_SZ},       // 3 
    // Autonomous modes 
    {"AUTO_MAX_PWM", &params.values.auto_max_pwm, MAV_PARAM_TYPE_REAL32, params.AUTO_MAX_PWM},   // 4 
    // Compass 
    {"COMPASS_TN",   &params.values.compass_tn,   MAV_PARAM_TYPE_REAL32, params.COMPASS_TN},     // 5 
    {"COMPASS_HIX",  &params.values.compass_hix,  MAV_PARAM_TYPE_REAL32, params.COMPASS_HIX},    // 6 
    {"COMPASS_HIY",  &params.values.compass_hiy,  MAV_PARAM_TYPE_REAL32, params.COMPASS_HIY},    // 7 
    {"COMPASS_HIZ",  &params.values.compass_hiz,  MAV_PARAM_TYPE_REAL32, params.COMPASS_HIZ},    // 8 
    {"COMPASS_SIDX", &params.values.compass_sidx, MAV_PARAM_TYPE_REAL32, params.COMPASS_SIDX},   // 9 
    {"COMPASS_SIDY", &params.values.compass_sidy, MAV_PARAM_TYPE_REAL32, params.COMPASS_SIDY},   // 10 
    {"COMPASS_SIDZ", &params.values.compass_sidz, MAV_PARAM_TYPE_REAL32, params.COMPASS_SIDZ},   // 11 
    {"COMPASS_SIOX", &params.values.compass_siox, MAV_PARAM_TYPE_REAL32, params.COMPASS_SIOX},   // 12 
    {"COMPASS_SIOY", &params.values.compass_sioy, MAV_PARAM_TYPE_REAL32, params.COMPASS_SIOY},   // 13 
    {"COMPASS_SIOZ", &params.values.compass_sioz, MAV_PARAM_TYPE_REAL32, params.COMPASS_SIOZ},   // 14 
    // Waypoints 
    {"WP_RADIUS",    &params.values.wp_radius,    MAV_PARAM_TYPE_REAL32, params.WP_RADIUS},      // 15 
    // Calculations 
    {"MADGWICK_B",   &params.values.madgwick_b,   MAV_PARAM_TYPE_REAL32, params.MADGWICK_B}      // 16 
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
        .param2 = vs_waypoint_radius,                 // Waypoint acceptance radius 
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
    
    //==================================================
    // The below code is temporary until parameters are fully implemented. 

    params.values.accel_sx = vs_accel_sx;
    params.values.accel_sy = vs_accel_sy;
    params.values.accel_sz = vs_accel_sz;

    params.values.auto_max_pwm = vs_auto_max_pwm;

    params.values.compass_tn = vs_tn_offset;
    
    params.values.compass_hix = vs_compass_hix;
    params.values.compass_hiy = vs_compass_hiy;
    params.values.compass_hiz = vs_compass_hiz;
    
    params.values.compass_sidx = vs_compass_sidx;
    params.values.compass_sidy = vs_compass_sidy;
    params.values.compass_sidz = vs_compass_sidz;
    
    params.values.compass_siox = vs_compass_siox;
    params.values.compass_sioy = vs_compass_sioy;
    params.values.compass_sioz = vs_compass_sioz;

    params.values.wp_radius = vs_waypoint_radius;

    params.values.madgwick_b = vs_madgwick_b;
    
    //==================================================
}


/**
 * @brief Establish and load vehicle info from external memory 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleMemory::MemoryLoad(Vehicle &vehicle)
{
    // Make sure the vehicle directory and its needed files exist. If they don't then 
    // create them and intialize the data. If they do then load exisiting data into the 
    // system. 

    vehicle.hardware.MemorySetDirectory();

    ParameterLoad(vehicle);
    MissionLoad();
}

//=======================================================================================


//=======================================================================================
// Parameters 

/**
 * @brief Load stored parameters if they exist 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleMemory::ParameterLoad(Vehicle &vehicle)
{
    // Once parameters are fully implemented, intialize this to zero and have it set 
    // once all parameters have been read. 
    static uint8_t parameters_read = FLAG_SET; 

    // As each parameter is read, perform a parameter set so values get updated 
    // throughout the code. 

    // Once all parameters have been read, set their values throughout the code. 
    if (parameters_read)
    {
        for (uint8_t i = RESET; i < num_parameters; i++)
        {
            ParameterSet(vehicle, parameters[i].name, *parameters[i].value); 
        }
    }
}


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
 * @param vehicle : vehicle object 
 * @param param_id : name/key/ID of parameter used to search for a parameter 
 * @param value : number to set parameter to if ID is valid 
 * @return ParamIndex : index of specified parameter - index == parameters.size() if invalid 
 */
ParamIndex VehicleMemory::ParameterSet(
    Vehicle &vehicle, 
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
        ParameterSetUpdate(vehicle, parameters[param_index].index); 
    }

    return param_index; 
}


/**
 * @brief Update parameter values within the autopilot as needed 
 * 
 * @param vehicle : vehicle object 
 * @param param_index : index of parameter to update 
 */
void VehicleMemory::ParameterSetUpdate(
    Vehicle &vehicle, 
    ParamIndex param_index)
{
    switch (param_index)
    {
        case Parameters::ACCEL_SX: 
            vehicle.navigation.AccelUncertaintyXSet(*parameters[param_index].value);
            break;

        case Parameters::ACCEL_SY: 
            vehicle.navigation.AccelUncertaintyYSet(*parameters[param_index].value);
            break;

        case Parameters::ACCEL_SZ: 
            vehicle.navigation.AccelUncertaintyZSet(*parameters[param_index].value);
            break;

        case Parameters::AUTO_MAX_PWM:
            vehicle.AutoDriveMaxPWMSet(static_cast<uint16_t>(*parameters[param_index].value));
            break;

        case Parameters::COMPASS_TN: 
            vehicle.navigation.TrueNorthOffsetSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_HIX:     
            vehicle.navigation.MagHardIronXSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_HIY:     
            vehicle.navigation.MagHardIronYSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_HIZ:     
            vehicle.navigation.MagHardIronZSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_SIDX:     
            vehicle.navigation.MagSoftIronDiagonalXSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_SIDY:     
            vehicle.navigation.MagSoftIronDiagonalYSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_SIDZ:     
            vehicle.navigation.MagSoftIronDiagonalZSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_SIOX:     
            vehicle.navigation.MagSoftIronOffDiagonalXSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_SIOY:     
            vehicle.navigation.MagSoftIronOffDiagonalYSet(*parameters[param_index].value);
            break;

        case Parameters::COMPASS_SIOZ:     
            vehicle.navigation.MagSoftIronOffDiagonalZSet(*parameters[param_index].value);
            break;
    
        case Parameters::WP_RADIUS: 
            vehicle.navigation.WaypointRadiusSet(*parameters[param_index].value);
            break;

        case Parameters::MADGWICK_B: 
            vehicle.navigation.MadgwickBetaSet(*parameters[param_index].value);
            break;
        
        default: 
            break;
    }
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
