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
public:   // public types 

    // Orientation 
    struct Vector 
    {
        int16_t x, y, z; 
    };

    // Position 
    struct Location 
    {
        int32_t latI, lonI, altI; 
        // float altF; 
        double lat, lon, alt; 
    };

private:   // Private members 

    double coordinate_lpf_gain;  // Low pass filter gain for GPS coordinates 
    int16_t true_north_offset;   // True north offset from magnetic north 
    
public:   // Public members 
    
    // Orientation 
    Vector accel, gyro, mag; 
    float roll, pitch, yaw; 
    int16_t heading, target_heading; 
    
    // Position 
    Location current, target, previous; 
    GPS_FIX_TYPE fix_type; 
    MAV_FRAME coordinate_frame; 
    uint16_t position_type_mask; 
    uint16_t waypoint_distance; 
    uint16_t ground_speed; 
    uint16_t num_satellite; 

private:   // private methods 

    // Position 
    void CoordinateFilter(Location new_data, Location &filtered_data) const; 
    int32_t GPSRadius(Location current, Location target); 
    int16_t GPSHeading(Location current, Location target); 
    int16_t TrueNorthHeading(int16_t heading) const; 
    int16_t HeadingError(int16_t current_heading, int16_t target_heading); 

public:   // public methods 

    // Constructor 
    VehicleNavigation(); 

    // Orientation 

    // Position 
    Location LocationCurrentGet(void); 
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
