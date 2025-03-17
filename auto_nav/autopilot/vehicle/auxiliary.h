/**
 * @file auxiliary.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle auxiliary systems interface 
 * 
 * @version 0.1
 * @date 2025-03-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _AUXILIARY_H_ 
#define _AUXILIARY_H_ 

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleAuxiliary 
{
public:   // public members 

    int16_t temperature; 
    uint32_t time_usec; 
}; 

//=======================================================================================

#endif   // _AUXILIARY_H_ 
