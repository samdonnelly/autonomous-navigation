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

#include "includes.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleNavigation 
{
public:   // Public members 

    // Orientation 
    struct Vector 
    {
        int16_t x, y, z; 
    }
    accel, gyro, mag; 

    float roll, pitch, yaw; 
    int16_t heading, target_heading; 

    // Position 
    struct Location 
    {
        int32_t lat, lon, alt; 
    }
    current, target, previous; 
    
    GPS_FIX_TYPE fix_type; 
    MAV_FRAME coordinate_frame; 
    uint16_t position_type_mask; 
    uint16_t waypoint_distance; 
    uint16_t ground_speed; 
    uint16_t num_satellite; 
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
