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

#include "tools.h" 
#include "nav_calcs.h" 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare the classes that will use the navigation module 

// Radio base class 
class NavModule : public nav_calculations 
{
private:   // Private members 

    // GPS 
    gps_waypoints_t current;           // Current location coordinates (filtered) 
    gps_waypoints_t target;            // Desired waypoint coordinates 
    uint8_t waypoint_index;            // Index of target waypoints 
    uint8_t num_gps_waypoints;         // Number of waypoints in the current mission 
    int32_t radius;                    // Distance between current and desired location 
    int32_t coordinate_radius;         // Threshold distance to target (meters*10) 

    // Heading 
    int16_t coordinate_heading;        // Heading between current and desired location 
    int16_t compass_heading;           // Current compass heading 

public:   // Public member functions 

    // Constructor(s) 
    NavModule(
        int32_t coord_radius, 
        double coordinate_filter_gain, 
        int16_t tn_offset) 
        : nav_calculations(coordinate_filter_gain, tn_offset), 
          waypoint_index(CLEAR), 
          num_gps_waypoints(CLEAR), 
          radius(coord_radius), 
          coordinate_radius(coord_radius), 
          coordinate_heading(CLEAR), 
          compass_heading(CLEAR) 
    {
        current.lat = CLEAR; 
        current.lon = CLEAR; 
        target.lat = CLEAR; 
        target.lon = CLEAR; 
    } 

    // Destructor 
    ~NavModule() {} 

    // Get the target waypoint 
    uint8_t GetTargetWaypoint(gps_waypoints_t& target_waypoint); 

    // Update coordinate radius 
    void SetCoordinateRadius(int32_t coord_radius); 

protected:   // Protected member functions 

    // Find the heading error 
    int16_t HeadingError(int16_t m_heading); 

    // Find the location error 
    void LocationError(
        gps_waypoints_t& position, 
        const gps_waypoints_t *waypoints); 

    // Update the waypoint mission index 
    uint8_t SetTargetWaypoint(
        uint8_t index, 
        const gps_waypoints_t *waypoints); 

    // Current location manual update 
    void SetCurrentLocation(const gps_waypoints_t& position); 

    // Update number of waypoints 
    void SetNumWaypoints(uint8_t num_waypoints); 

private:   // Private member functions 

    // Target waypoint update 
    void SetTargetLocation(const gps_waypoints_t& waypoint); 
}; 

//=======================================================================================

#endif   // _NAV_MODULE_H_ 
