/**
 * @file memory.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle memory interface 
 * 
 * @version 0.1
 * @date 2025-03-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _MEMORY_H_ 
#define _MEMORY_H_ 

//=======================================================================================
// Includes 

#include "includes.h" 

//=======================================================================================


//=======================================================================================
// Macros 

constexpr uint16_t memory_buff_size = 250;   // External memory individual file data size 

#define MAX_MISSION_SIZE 20   // Max number of mission items 
#define HOME_OFFSET 1         // First mission item that isn't the home location 
#define HOME_INDEX 0          // Mission item index for the home location 

//=======================================================================================


//=======================================================================================
// Datatypes 

typedef uint8_t ParamIndex;
typedef mavlink_mission_item_int_t MissionItem;
typedef uint16_t MissionIndex, MissionSize;

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle;


class VehicleMemory 
{
public:

    enum ParameterList : uint16_t 
    {
        // Accelerometer 
        ACCEL_SX,        // Uncertainty/variance along the X axis (g's) 
        ACCEL_SY,        // Uncertainty/variance along the Y axis (g's) 
        ACCEL_SZ,        // Uncertainty/variance along the Z axis (g's) 
        // Autonomous modes 
        AUTO_MAX_PWM,    // Max PWM output of motor(s) in autonomous modes 
        // Compass 
        COMPASS_TN,      // True North offset (magnetic declination) 
        COMPASS_HIX,     // Hard iron offset on the X axis (milligauss) 
        COMPASS_HIY,     // Hard iron offset on the Y axis (milligauss) 
        COMPASS_HIZ,     // Hard iron offset on the Z axis (milligauss) 
        COMPASS_SIDX,    // Soft iron diagonal X axis component 
        COMPASS_SIDY,    // Soft iron diagonal Y axis component 
        COMPASS_SIDZ,    // Soft iron diagonal Z axis component 
        COMPASS_SIOX,    // Soft iron off-diagonal X axis component 
        COMPASS_SIOY,    // Soft iron off-diagonal Y axis component 
        COMPASS_SIOZ,    // Soft iron off-diagonal Z axis component 
        // Logging 
        LOG_ENABLE,      // Enable data logging 
        LOG_DIVIDER,     // Log interval divider 
        // Waypoints 
        WP_RADIUS,       // Waypoint radius 
        // Calculations 
        MADGWICK_B,      // Madgwick filter beta (weighted correction) 

        NUM_PARAMETERS   // Number of parameters in the system 
    };

    // Parameter information 
    struct ParamInfo 
    {
        const char *name;           // Name/Key/ID of parameter - used to match/identify 
        float *value;               // Value of parameter that gets used in the system 
        const MAV_PARAM_TYPE type;  // Parameter datatype - See AUTOPILOT_VERSION.capabilities 
        const ParamIndex index;     // Index of parameter in list - used to call setters 
    };
    
    // Mission information 
    struct MissionInfo 
    {
        std::array<MissionItem, MAX_MISSION_SIZE + HOME_OFFSET> items;
        uint16_t target;  // Target mission item index 
        uint16_t size;    // Number of mission items 
        uint32_t id;      // ID number of stored mission 
        uint8_t type;     // Type of the stored mission 
    };

    /**
     * @brief Constructor 
     */
    VehicleMemory();

    /**
     * @brief Destructor 
     */
    ~VehicleMemory() = default;

    // General 
    void MemoryLoad(Vehicle &vehicle);

    // Parameters 
    ParamIndex ParameterSet(Vehicle &vehicle, const char *param_id, float &value);
    ParamIndex ParameterLookUp(const char *param_id);
    bool ParameterIndexCheck(uint8_t index);

    // Mission 
    MissionItem MissionItemGet(uint16_t sequence);
    void MissionItemSet(MissionItem &mission_item);
    void MissionHomeLocationSet(int32_t lat, int32_t lon, float alt);
    bool MissionHomeLocationStatus(void);
    MissionIndex MissionTargetGet(void);
    bool MissionTargetSet(uint16_t sequence);
    MissionSize MissionSizeGet(void);
    void MissionSizeSet(uint16_t size);
    uint8_t MissionTypeGet(void);
    void MissionTypeSet(uint8_t type);
    uint32_t MissionIDGet(void);
    uint32_t MissionIDUpdate(void);
    void MissionClear(void);

    // Data Logging 
    void LogData(Vehicle &vehicle);

private:

    union Status
    {
        struct 
        {
            uint32_t home_location : 1;
            uint32_t log_file_open : 1;
        };
        uint32_t flags;
    }
    status;

    // Mission 
    MissionInfo mission;

    // Data logging 
    uint16_t log_divider_count;
    uint16_t log_divider_limit;

    // Parameters 
    void ParameterLoad(Vehicle &vehicle);
    void ParameterReadAll(Vehicle &vehicle);
    void ParameterWriteAll(Vehicle &vehicle);
    void ParameterSave(Vehicle &vehicle, const char *param_id, float &value);
    void ParameterStrFormat(char *param_buff, uint16_t buff_size, const char *param_id, float &value);
    void ParameterSetUpdate(Vehicle &vehicle, ParamIndex param_index);

    // Mission 
    void MissionLoad(void);

    // Helper functions 
    void ExternalMemoryEventQueue(Vehicle &vehicle, Event event);
};

//=======================================================================================


//=======================================================================================
// Other data 

extern const std::array<VehicleMemory::ParamInfo, VehicleMemory::NUM_PARAMETERS> parameters;

//=======================================================================================

#endif   // _MEMORY_H_ 
