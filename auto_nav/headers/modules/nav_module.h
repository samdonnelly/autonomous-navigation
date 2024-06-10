/**
 * @file nav_module.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Navigation module interface 
 * 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _NAV_MODULE_H_
#define _NAV_MODULE_H_ 

//=======================================================================================
// Includes 

#include "includes_cpp_drivers.h" 

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare the classes that will use the navigation module 

// Radio base class 
class NavModule : public nav_calculations 
{
protected:   // Protected members 

    // 

private:   // Private member functions 

    // GPS 
    gps_waypoints_t current;           // Current location coordinates 
    gps_waypoints_t target;            // Desired waypoint coordinates 
    uint8_t waypoint_index;            // Index of target waypoints 
    uint8_t num_gps_waypoints;         // Number of waypoints in the current mission 
    int32_t radius;                    // Distance between current and desired location 
    int32_t coordinate_radius;         // Threshold distance to target (meters*10) 

    // Heading 
    int16_t coordinate_heading;        // Heading between current and desired location 
    int16_t compass_heading;           // Current compass heading 
    int16_t error_heading;             // Error between compass and coordinate heading 

protected:   // Protected member functions 

    // 

public:   // Public member functions 

    // Constructor(s) 
    NavModule(int32_t coord_radius) 
        : waypoint_index(CLEAR), 
          num_gps_waypoints(CLEAR), 
          radius(CLEAR), 
          coordinate_radius(coord_radius), 
          coordinate_heading(CLEAR), 
          compass_heading(CLEAR), 
          error_heading(CLEAR) 
    {
        current.lat = CLEAR; 
        current.lon = CLEAR; 
    } 

    // Destructor 
    ~NavModule() {} 

    // Find the heading error 
    int16_t HeadingError(int16_t m_heading); 

    // Find the location error 
    void LocationError(
        gps_waypoints_t& position, 
        const gps_waypoints_t *waypoints); 

    // Update coordinate radius 
    void SetCoordinateRadius(int32_t coord_radius); 

    // Update number of waypoints 
    void SetNumWaypoints(uint8_t num_waypoints); 
}; 

//=======================================================================================

#endif   // _NAV_MODULE_H_ 
