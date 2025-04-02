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

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleMemory 
{
public:   // public members 

    // Parameters 
    struct ParamInfo 
    {
        const char *name; 
        float *value; 
        // Mission planner seems to only accept floats so 
        // no option for MAV_PARAM_TYPE is offered here. 
    };

    uint8_t param_index; 
    MAV_PARAM_TYPE param_value_type; 

    // Mission - this may change as external storage is integrated 
    uint16_t mission_size; 
    uint32_t mission_id; 
    uint8_t mission_type; 
    uint16_t mission_index; 
    mavlink_mission_item_int_t mission[MAX_MISSION_SIZE]; 

public:   // public methods 

    // Constructor 
    VehicleMemory(); 

    // Parameters 
    void ParameterLookUp(char *param_id); 
    void ParameterSet(char *param_id, float &value); 

    // Mission 
    void MissionLoad(void); 
}; 

//=======================================================================================


//=======================================================================================
// Other data 

extern const std::array<VehicleMemory::ParamInfo, NUM_PARAMETERS> parameters; 

//=======================================================================================

#endif   // _MEMORY_H_ 
