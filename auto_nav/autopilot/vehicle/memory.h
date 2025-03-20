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
// Classes 

class VehicleMemory 
{
public:   // public members 

    // Parameters 
    struct ParamInfo 
    {
        const char *name; 
        float value; 
        // Mission planner seems to only accept floats so 
        // no option for MAV_PARAM_TYPE is offered here. 
    };

    uint8_t param_index; 
    uint8_t num_params; 
    MAV_PARAM_TYPE param_value_type; 

    // Mission 
    uint16_t mission_size; 
    uint32_t mission_id; 

public:   // public methods 

    VehicleMemory(); 
}; 

//=======================================================================================

extern const VehicleMemory::ParamInfo parameters[]; 

#endif   // _MEMORY_H_ 
