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

#define NUM_PARAMETERS 2      // Number of parameters in the system 
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
public:   // public types 

    // Parameters 
    struct ParamInfo 
    {
        const char *name;            // Name/Key/ID of parameter - used to match/identify 
        float *value;                // Value of parameter that gets used in the system 
        const MAV_PARAM_TYPE type;   // Parameter datatype - See AUTOPILOT_VERSION.capabilities 
        const ParamIndex index;      // Index of parameter in list - used to call setters 
    };
    
    // Mission 
    struct MissionInfo 
    {
        std::array<MissionItem, MAX_MISSION_SIZE + HOME_OFFSET> items; 
        uint16_t target;   // Target mission item index 
        uint16_t size;     // Number of mission items 
        uint32_t id;       // ID number of stored mission 
        uint8_t type;      // Type of the stored mission 
    }; 

private:   // private members 

    union Status
    {
        struct 
        {
            uint32_t home_location : 1; 
        }; 
        uint32_t flags; 
    }
    status; 

    // Mission 
    MissionInfo mission; 

private:   // private methods 

    // Parameters 
    void ParameterSetUpdate(Vehicle &vehicle, ParamIndex param_index); 

public:   // public methods 

    // Constructor 
    VehicleMemory(); 

    // Parameters 
    void ParameterLoad(Vehicle &vehicle); 
    bool ParameterIndexCheck(uint8_t index); 
    ParamIndex ParameterLookUp(const char *param_id); 
    ParamIndex ParameterSet(Vehicle &vehicle, const char *param_id, float &value); 

    // Mission 
    void MissionLoad(void); 
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
}; 

//=======================================================================================


//=======================================================================================
// Other data 

extern const std::array<VehicleMemory::ParamInfo, NUM_PARAMETERS> parameters; 

//=======================================================================================

#endif   // _MEMORY_H_ 
