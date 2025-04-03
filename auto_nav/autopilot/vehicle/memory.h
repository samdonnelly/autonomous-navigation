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

#define NUM_PARAMETERS 5 
#define MAX_MISSION_SIZE 10 
#define HOME_OFFSET 1 
#define HOME_INDEX 0 

//=======================================================================================


//=======================================================================================
// Datatypes 

typedef mavlink_mission_item_int_t MissionItem; 

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleMemory 
{
public:   // public types 

    // Parameters 
    struct ParamInfo 
    {
        const char *name;   // Name/Key/ID of parameter - used to match/identify 
        float *value;       // Value of parameter that gets used in the system 
        // Mission planner seems to only accept floats so 
        // no option for MAV_PARAM_TYPE is offered here. 
    };
    
    // Mission 
    struct MissionInfo 
    {
        uint16_t mission_size;    // Number of mission items (not including home) 
        uint16_t mission_index;   // Mission item being executed (not including home) 
        uint32_t mission_id;      // ID number of stored mission 
        uint8_t mission_type;     // Type of the stored mission 
    }; 

private:   // private members 

    // Mission - this may change as external storage is integrated 
    std::array<MissionItem, MAX_MISSION_SIZE + HOME_OFFSET> mission; 
    // mavlink_mission_item_int_t mission[MAX_MISSION_SIZE + HOME_OFFSET]; 

public:   // public members 

    // Parameters 
    uint8_t param_index; 
    MAV_PARAM_TYPE param_value_type; 

    // Mission - this may change as external storage is integrated 
    uint16_t mission_size; 
    uint16_t mission_index; 
    uint32_t mission_id; 
    uint8_t mission_type; 
    MissionInfo mission_info; 

public:   // public methods 

    // Constructor 
    VehicleMemory(); 

    // Parameters 
    void ParameterLookUp(char *param_id); 
    void ParameterSet(char *param_id, float &value); 

    // Mission 
    void MissionLoad(void); 
    MissionItem MissionItemGet(uint16_t sequence); 
    MissionItem MissionHomeGet(void); 
    void MissionItemSet(uint16_t sequence, MissionItem &mission_item); 
    void MissionHomeSet(MissionItem &mission_item); 
    bool MissionTargetSet(uint16_t sequence); 
    void MissionClear(void); 
}; 

//=======================================================================================


//=======================================================================================
// Other data 

extern const std::array<VehicleMemory::ParamInfo, NUM_PARAMETERS> parameters; 

//=======================================================================================

#endif   // _MEMORY_H_ 
