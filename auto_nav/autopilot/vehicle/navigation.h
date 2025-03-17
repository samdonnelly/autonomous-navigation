/**
 * @file navigation.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle navigation interface 
 * 
 * @version 0.1
 * @date 2025-03-13
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _NAVIGATION_H_ 
#define _NAVIGATION_H_ 

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleNavigation 
{
public:   // Public members 

    int16_t accel[3]; 
    int16_t gyro[3]; 
    int16_t mag[3]; 
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
