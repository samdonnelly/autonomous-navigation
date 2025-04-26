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
// Macros 

#define DEGREE_DATATYPE 10000000   // int32_t <--> float - removes or adds decimal places 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle; 


class VehicleNavigation 
{
public:   // public types 

    struct Location 
    {
        int32_t latI, lonI, altI; 
        float lat, lon, alt; 
    };

    template <typename T>
    struct Vector 
    {
        T x, y, z; 
    };

private:   // private members 

    struct Timers 
    {
        uint8_t gps_connection; 
        uint8_t imu_connection; 
    }
    timers; 

    union Status
    {
        struct 
        {
            uint32_t gps_connected     : 1; 
            uint32_t gps_status_change : 1; 
            uint32_t imu_connected     : 1; 
        }; 
        uint32_t flags; 
    }
    status; 

    // Location 
    Location location_current, location_filtered, location_target; 
    mavlink_mission_item_int_t mission_target; 

    float coordinate_lpf_gain;   // Low pass filter gain for GPS coordinates 

    // Orientation 
    Vector<int16_t> accel, gyro, mag; 
    Vector<float> orient;               // x = roll, y = pitch, z = yaw 
    int16_t heading, heading_target; 
    int16_t true_north_offset;          // True north offset from magnetic north 
    
public:   // public members 
    
    // Location 
    Location location_previous; 
    GPS_FIX_TYPE fix_type; 
    MAV_FRAME coordinate_frame; 
    uint16_t position_type_mask; 
    uint16_t waypoint_distance; 
    uint16_t ground_speed; 
    uint16_t num_satellite; 

private:   // private methods 

    // Navigation data handling 
    void LocationChecks(Vehicle &vehicle); 
    void OrientationChecks(Vehicle &vehicle); 
    
    // Mission execution 
    void TargetUpdate(Vehicle &vehicle); 
    
    // Location calculations 
    void WaypointDistance(Vehicle &vehicle); 
    void CoordinateFilter(Location new_location, Location &filtered_location) const; 
    // int32_t GPSRadius(Location current, Location target); 
    float GPSRadius(Location current, Location target); 
    int16_t GPSHeading(Location current, Location target); 

    // Heading calculations 
    int16_t TrueNorthHeading(int16_t heading) const; 
    int16_t HeadingError(int16_t current_heading, int16_t target_heading); 

public:   // public methods 

    // Constructor 
    VehicleNavigation(); 

    // Navigation data handling 
    void LocationUpdate(Vehicle &vehicle); 
    void OrientationUpdate(Vehicle &vehicle); 
    
    // Mission execution 
    void TargetAssess(Vehicle &vehicle); 
    void CourseCorrection(Vehicle &vehicle); 

    // Getters and setter 
    Location LocationCurrentGet(void); 
    VehicleNavigation::Vector<int16_t> AccelCurrentGet(void); 
    VehicleNavigation::Vector<int16_t> GyroCurrentGet(void); 
    VehicleNavigation::Vector<int16_t> MagCurrentGet(void); 
    VehicleNavigation::Vector<float> OrientationCurrentGet(void); 
    int16_t HeadingCurrentGet(void); 
    int16_t HeadingTargetGet(void); 
}; 

//=======================================================================================

#endif   // _NAVIGATION_H_ 
