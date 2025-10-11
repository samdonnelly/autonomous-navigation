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
// Constants 

constexpr char parameters_filename[] = "parameters.txt";
constexpr char parameter_read_format[] = "%s %s";
constexpr int parm_read_num_items = 2;
constexpr char parameter_write_format[] = "%s %ld.%ld     ";
constexpr float param_scalar = 1000.0f;

//=======================================================================================


//=======================================================================================
// Parameters 

struct ParameterValues
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
    // Logging 
    log_enable,     // Enable data logging 
    log_divider,    // Log interval divider 
    // Waypoints 
    wp_radius,      // Waypoint radius (meters) 
    // Calculations 
    madgwick_b;     // Madgwick filter beta (weighted correction) 
};

static ParameterValues param_values;


// The following parameters are available across the vehicle system but cannot be 
// modified outside of this file. Only parameter values can be changed and it's done 
// using the ParameterValues struct. 

const std::array<VehicleMemory::ParamInfo, VehicleMemory::NUM_PARAMETERS> parameters = 
{{
    // Accelerometer 
    {"ACCEL_SX",     &param_values.accel_sx,     MAV_PARAM_TYPE_REAL32, VehicleMemory::ACCEL_SX},
    {"ACCEL_SY",     &param_values.accel_sy,     MAV_PARAM_TYPE_REAL32, VehicleMemory::ACCEL_SY},
    {"ACCEL_SZ",     &param_values.accel_sz,     MAV_PARAM_TYPE_REAL32, VehicleMemory::ACCEL_SZ},
    // Autonomous modes 
    {"AUTO_MAX_PWM", &param_values.auto_max_pwm, MAV_PARAM_TYPE_REAL32, VehicleMemory::AUTO_MAX_PWM},
    // Compass 
    {"COMPASS_TN",   &param_values.compass_tn,   MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_TN},
    {"COMPASS_HIX",  &param_values.compass_hix,  MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_HIX},
    {"COMPASS_HIY",  &param_values.compass_hiy,  MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_HIY},
    {"COMPASS_HIZ",  &param_values.compass_hiz,  MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_HIZ},
    {"COMPASS_SIDX", &param_values.compass_sidx, MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_SIDX},
    {"COMPASS_SIDY", &param_values.compass_sidy, MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_SIDY},
    {"COMPASS_SIDZ", &param_values.compass_sidz, MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_SIDZ},
    {"COMPASS_SIOX", &param_values.compass_siox, MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_SIOX},
    {"COMPASS_SIOY", &param_values.compass_sioy, MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_SIOY},
    {"COMPASS_SIOZ", &param_values.compass_sioz, MAV_PARAM_TYPE_REAL32, VehicleMemory::COMPASS_SIOZ},
    // Logging 
    {"LOG_ENABLE",   &param_values.log_enable,   MAV_PARAM_TYPE_REAL32, VehicleMemory::LOG_ENABLE},
    {"LOG_DIVIDER",  &param_values.log_divider,  MAV_PARAM_TYPE_REAL32, VehicleMemory::LOG_DIVIDER},
    // Waypoints 
    {"WP_RADIUS",    &param_values.wp_radius,    MAV_PARAM_TYPE_REAL32, VehicleMemory::WP_RADIUS},
    // Calculations 
    {"MADGWICK_B",   &param_values.madgwick_b,   MAV_PARAM_TYPE_REAL32, VehicleMemory::MADGWICK_B}
}};

//=======================================================================================


//=======================================================================================
// Initialization 

VehicleMemory::VehicleMemory()
    : status(),
      log_divider_count(RESET),
      log_divider_limit(RESET)
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
    
    //==================================================
    // The below code is temporary until parameters are fully implemented. 

    param_values.accel_sx = vs_accel_sx;
    param_values.accel_sy = vs_accel_sy;
    param_values.accel_sz = vs_accel_sz;

    param_values.auto_max_pwm = vs_auto_max_pwm;

    param_values.compass_tn = vs_tn_offset;
    
    param_values.compass_hix = vs_compass_hix;
    param_values.compass_hiy = vs_compass_hiy;
    param_values.compass_hiz = vs_compass_hiz;
    
    param_values.compass_sidx = vs_compass_sidx;
    param_values.compass_sidy = vs_compass_sidy;
    param_values.compass_sidz = vs_compass_sidz;
    
    param_values.compass_siox = vs_compass_siox;
    param_values.compass_sioy = vs_compass_sioy;
    param_values.compass_sioz = vs_compass_sioz;

    param_values.wp_radius = vs_waypoint_radius;

    param_values.madgwick_b = vs_madgwick_b;
    
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

    // Queue MEMORY_SETUP then wait until it finishes before proceeding. 
    ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_SETUP));

    // Check status of memory before proceeding 
    if (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_OK)
    {
        ParameterLoad(vehicle);
        MissionLoad();
    }
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
    // Set the file name to be the parameters file 
    vehicle.hardware.MemorySetFileName(parameters_filename, 
                                       static_cast<uint16_t>(sizeof(parameters_filename)));
    
    // Attempt to open the parameters file. If it's found that the parameter file already 
    // exists then the code reads the parameters from external memory and populates the 
    // parameter values in the code. If it's found that the parameter file did not exist 
    // and it needed to be created, then the code write the default parameter values to 
    // the file which can then be updated as needed. 
    ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_OPEN));

    if (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_FILE_OPENED)
    {
        ParameterReadAll(vehicle);
    }
    else if (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_FILE_CREATED)
    {
        ParameterWriteAll(vehicle);
    }

    // Attempt to close the parameters file 
    ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_CLOSE));

    // Once all parameters have been established, set their values throughout the code. 
    for (uint16_t i = RESET; i < NUM_PARAMETERS; i++)
    {
        ParameterSetUpdate(vehicle, parameters[i].index);
    }
}


/**
 * @brief Read all parameters from external memory 
 * 
 * @details This function reads parameters from an already open parameters file in 
 *          external memory and uses the values found to populate the parameters values 
 *          in the code. If there's a mismatch found between code and memory parameters, 
 *          then the parameters file gets erased and rewritten so that they align 
 *          (previously stored values will not be lost). 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleMemory::ParameterReadAll(Vehicle &vehicle)
{
    char param_buff[memory_buff_size];
    char param_name[memory_buff_size], param_value_str[memory_buff_size];
    uint16_t item_count = RESET;
    uint16_t checksum_1 = RESET, checksum_2 = RESET;

    ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_READ));
    
    while (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_OK)
    {
        vehicle.hardware.MemoryGetData(param_buff, memory_buff_size);
        int scan_result = sscanf(param_buff, parameter_read_format, param_name, param_value_str);
        item_count++;

        if (scan_result == parm_read_num_items)
        {
            // If the scan is successful then we check for a parameter to update. 
            float param_value = strtof(param_value_str, nullptr);
            ParamIndex param_index = ParameterLookUp(param_name);

            if (ParameterIndexCheck(param_index))
            {
                *parameters[param_index].value = param_value;

                // This is used to see if all parametrs get updated. 
                checksum_1 += param_index;
                checksum_2 ^= param_index;
            }
        }

        ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_READ));
    }

    if (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_EOF)
    {
        // There are two mechanisms used to keep track of the parameters in the code 
        // and the parameters saved in external memory. The first are checksums of 
        // all the parameters indexes and the second is a total item count within the 
        // external memory parameter file. These two together ensure the parameters 
        // and their values in both the code and in external memory match. These 
        // prevent against cases such as there being less parameters saved in memory 
        // than in the code (parameters added), more parameters saved in memory than 
        // in the code (parameters removed), the same number of parameters in both 
        // places but parameter names changed or parameters replaced by others, and 
        // duplicates of the same parameter saved in external memory. 

        uint16_t checksum_check_1 = RESET, checksum_check_2 = RESET;

        // Find the expected checksums of the parameter indexes 
        for (uint16_t i = RESET; i < NUM_PARAMETERS; i++)
        {
            checksum_check_1 += i;
            checksum_check_2 ^= i;
        }

        if ((checksum_1 != checksum_check_1) || 
            (checksum_2 != checksum_check_2) || 
            (item_count != NUM_PARAMETERS))
        {
            // There's a mismatch between the parameters in code and in external 
            // memory. Go to the beginning of the file, remove all previous data 
            // and write the new data. 
            vehicle.hardware.MemoryFilePositionSet(RESET);
            ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_NAVIGATE));
            ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_TRUNCATE));
            ParameterWriteAll(vehicle);
        }
    }
}


/**
 * @brief Write all parameters to external memory 
 * 
 * @details This function writes parameters to an already open parameters file in 
 *          external memory using the parameter values found in the code. 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleMemory::ParameterWriteAll(Vehicle &vehicle)
{
    char param_buff[memory_buff_size];

    // Go through each parameter in the code and save it to external memory 
    for (uint16_t i = RESET; i < NUM_PARAMETERS; i++)
    {
        ParameterStrFormat(param_buff, memory_buff_size, parameters[i].name, *parameters[i].value);
        vehicle.hardware.MemorySetData(param_buff, memory_buff_size);
        ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_WRITE));

        if (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_ACCESS_ERROR)
        {
            break;
        }
    }
}


/**
 * @brief Update the parameter that matches the ID/name 
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

        ParameterSave(vehicle, param_id, value);
        *parameters[param_index].value = value; 
        ParameterSetUpdate(vehicle, parameters[param_index].index);
    }

    return param_index; 
}


/**
 * @brief Find a parameters index using the ID/name 
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
 * @brief Check if an index is within the total number of parameters 
 * 
 * @param param_index : index of parameter within 'parameters' 
 * @return true/false : status of the check - true if within range 
 */
bool VehicleMemory::ParameterIndexCheck(uint8_t param_index)
{
    return (param_index < parameters.size()); 
}


/**
 * @brief Write one parameter to external memory 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleMemory::ParameterSave(
    Vehicle &vehicle, 
    const char *param_id, 
    float &value)
{
    // Set the parameter file name and attempt to open the file. If there are no issues 
    // opening the file, then start searching for where to write the parameter value. 
    vehicle.hardware.MemorySetFileName(parameters_filename, 
                                       static_cast<uint16_t>(sizeof(parameters_filename)));
    ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_OPEN));

    if (vehicle.external_memory_status != VehicleHardware::MemoryStatus::MEMORY_ACCESS_ERROR)
    {
        char param_buff[memory_buff_size];
        char param_name[memory_buff_size], param_value_str[memory_buff_size];
        uint32_t file_position = RESET;

        // Read the parameters from external memory one at a time. Check the parameter ID 
        // read against the specified parameter ID to update. If a match is found then 
        // overwrite the saved value in external memory with the new value. 
        ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_READ));
        
        while (vehicle.external_memory_status == VehicleHardware::MemoryStatus::MEMORY_OK)
        {
            vehicle.hardware.MemoryGetData(param_buff, memory_buff_size);
            sscanf(param_buff, parameter_read_format, param_name, param_value_str);

            if (strcmp(param_name, param_id) == 0)
            {
                // Parameter matched. Format the ID and value into a string then stage it 
                // for writing to external memory. 
                ParameterStrFormat(param_buff, memory_buff_size, param_id, value);
                vehicle.hardware.MemorySetData(param_buff, memory_buff_size);
                
                // After finding a match, the file needs to be backtracked to be able to 
                // overwrite the matched parameter that was just read. This is done by 
                // keeping track of where the matching parameter starts within the file. 
                // The position is file is set then the new parameter string is written. 
                vehicle.hardware.MemoryFilePositionSet(file_position);
                ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_NAVIGATE));
                ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_WRITE));

                break;
            }
            
            file_position += static_cast<uint32_t>(strlen(param_buff));
            ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_READ));
        }
    }

    // Close the file. 
    ExternalMemoryEventQueue(vehicle, static_cast<Event>(Vehicle::CommsEvents::MEMORY_CLOSE));
}


/**
 * @brief Format a parameter ID and value into a string for writing to external memory 
 * 
 * @param param_buff : buffer to store parameter string 
 * @param buff_size : size of buffer to store parameter string 
 * @param param_id : ID/name of parameter 
 * @param value : value of parameter 
 */
void VehicleMemory::ParameterStrFormat(
    char *param_buff, 
    uint16_t buff_size, 
    const char *param_id, 
    float &value)
{
    int32_t float_int = static_cast<int32_t>(value);
    float decimal = static_cast<float>(float_int) - value;
    int32_t float_dec = abs(static_cast<int32_t>(decimal * param_scalar));

    snprintf(param_buff, 
             buff_size, 
             parameter_write_format, 
             param_id, float_int, float_dec);
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
        case ACCEL_SX:
            vehicle.navigation.AccelUncertaintyXSet(*parameters[param_index].value);
            break;

        case ACCEL_SY:
            vehicle.navigation.AccelUncertaintyYSet(*parameters[param_index].value);
            break;

        case ACCEL_SZ:
            vehicle.navigation.AccelUncertaintyZSet(*parameters[param_index].value);
            break;

        case AUTO_MAX_PWM:
            vehicle.AutoDriveMaxPWMSet(static_cast<uint16_t>(*parameters[param_index].value));
            break;

        case COMPASS_TN:
            vehicle.navigation.TrueNorthOffsetSet(*parameters[param_index].value);
            break;

        case COMPASS_HIX:
            vehicle.navigation.MagHardIronXSet(*parameters[param_index].value);
            break;

        case COMPASS_HIY:
            vehicle.navigation.MagHardIronYSet(*parameters[param_index].value);
            break;

        case COMPASS_HIZ:
            vehicle.navigation.MagHardIronZSet(*parameters[param_index].value);
            break;

        case COMPASS_SIDX:
            vehicle.navigation.MagSoftIronDiagonalXSet(*parameters[param_index].value);
            break;

        case COMPASS_SIDY:
            vehicle.navigation.MagSoftIronDiagonalYSet(*parameters[param_index].value);
            break;

        case COMPASS_SIDZ:
            vehicle.navigation.MagSoftIronDiagonalZSet(*parameters[param_index].value);
            break;

        case COMPASS_SIOX:
            vehicle.navigation.MagSoftIronOffDiagonalXSet(*parameters[param_index].value);
            break;

        case COMPASS_SIOY:
            vehicle.navigation.MagSoftIronOffDiagonalYSet(*parameters[param_index].value);
            break;

        case COMPASS_SIOZ:
            vehicle.navigation.MagSoftIronOffDiagonalZSet(*parameters[param_index].value);
            break;

        case LOG_DIVIDER:
            log_divider_limit = static_cast<uint16_t>(*parameters[param_index].value);
            break;
    
        case WP_RADIUS:
            vehicle.navigation.WaypointRadiusSet(*parameters[param_index].value);
            break;

        case MADGWICK_B:
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

    // The MAVLink library can be used to format and decode mission items. 
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


//=======================================================================================
// Data logging 

// Create the log file (state entry process). 
// What do we do when there's a file name conflict? (ex. starting logs without GPS so 
// file name has a blank file stamp). 


/**
 * @brief Log data to external memory 
 * 
 * @param vehicle : vehicle object 
 */
void VehicleMemory::LogData(Vehicle &vehicle)
{
    // Check for both an open file and the log interval divider 
    if ((status.log_file_open == FLAG_SET) && (++log_divider_count >= log_divider_limit))
    {
        log_divider_count = RESET;

        // Get needed data 
        // Format data into a string 
        // Stage the string of data for writing to external memory 
        // Queue an external memory write event 
    }
}


// Close the log file (state exit process). 

//=======================================================================================


//=======================================================================================
// Helper functions 

// File transition 
// Check for an open file. If one is not open then proceed to open the needed file. If 
// one is open then first close it and make note of which file was open before opening 
// the new file. 
// Call this function again to reverse the open file (how do we keep track of the file 
// that needs to be reopened?). Once reopened, go to the end of the file so things can 
// pick up where they left off. 

/**
 * @brief Queue an external memory event 
 * 
 * @param vehicle : vehicle object 
 * @param event : comms event index 
 */
void VehicleMemory::ExternalMemoryEventQueue(Vehicle &vehicle, Event event)
{
    vehicle.CommsEventQueue(event);
    
    // External memory hardware communication sometimes needs to happen before the main 
    // thread can proceed. This is done through a semaphore which gets initialized to 0 
    // and the main thread waits on the Sempahore relase before it proceeds. The comms 
    // thread will release the semaphore and once the main thread aquires it, it gives 
    // it right back since it only needed it in order to proceed. 
    osSemaphoreAcquire(vehicle.external_memory_semaphore, portMAX_DELAY);
    osSemaphoreRelease(vehicle.external_memory_semaphore);
}

//=======================================================================================
