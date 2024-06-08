/**
 * @file nav_module.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Navigation module 
 * 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "nav_module.h" 

//=======================================================================================


//=======================================================================================
// User functions 

// Find the heading error 
int16_t NavModule::HeadingError(int16_t m_heading)
{
    // Determine the true north heading and find the 
    // error between the current (compass) and desired (GPS) headings. 
    compass_heading = true_north_heading(m_heading); 
    error_heading = heading_error(compass_heading, coordinate_heading); 

    return error_heading; 
}


// Find the location error 
void NavModule::LocationError(
    gps_waypoints_t& position, 
    const gps_waypoints_t *targets)
{
    // Get the updated location by reading the GPS device coordinates then filtering 
    // the result. 
    coordinate_filter(position, current); 

    // Calculate the distance to the target location and the heading needed to get 
    // there. 
    radius = gps_radius(current, target); 
    coordinate_heading = gps_heading(current, target); 

    // Check if the distance to the target is within the threshold. If so, the 
    // target is considered "hit" and we can move to the next target. 
    if (radius < coordinate_radius)
    {
        // Adjust waypoint index 
        if (++waypoint_index >= num_gps_waypoints)
        {
            waypoint_index = CLEAR; 
        }

        // Update the target waypoint 
        target.lat = targets[waypoint_index].lat; 
        target.lon = targets[waypoint_index].lon; 
    }
}


// Update coordinate radius 
void NavModule::SetCoordinateRadius(int32_t coord_radius)
{
    coordinate_radius = coord_radius; 
}


// Update number of waypoints 
void NavModule::SetNumWaypoints(uint8_t num_waypoints)
{
    num_gps_waypoints = num_waypoints; 
}


// Manual waypoint target update? (for index radio command) 

//=======================================================================================
