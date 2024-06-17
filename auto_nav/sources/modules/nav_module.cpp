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
// Vehicle functions 

// Find the heading error 
int16_t NavModule::HeadingError(int16_t m_heading)
{
    // Determine the true north heading and find the error between the current (compass) 
    // and desired (GPS) headings. 
    compass_heading = true_north_heading(m_heading); 
    return heading_error(compass_heading, coordinate_heading); 
}


// Find the location error 
void NavModule::LocationError(
    gps_waypoints_t& position, 
    const gps_waypoints_t *waypoints)
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
        SetTargetLocation(waypoints[waypoint_index]); 
    }
}


// Update the waypoint mission index 
uint8_t NavModule::SetTargetWaypoint(
    uint8_t index, 
    const gps_waypoints_t *waypoints)
{
    if (index < num_gps_waypoints)
    {
        waypoint_index = index; 
        SetTargetLocation(waypoints[waypoint_index]); 
        return TRUE; 
    }

    return FALSE; 
}


// Current location manual update 
void NavModule::SetCurrentLocation(const gps_waypoints_t& position)
{
    current.lat = position.lat; 
    current.lon = position.lon; 
    coordinate_heading = gps_heading(current, target); 
}


// Update number of waypoints 
void NavModule::SetNumWaypoints(uint8_t num_waypoints)
{
    num_gps_waypoints = num_waypoints; 
}

//=======================================================================================


//=======================================================================================
// System functions 

// Get the target waypoint 
uint8_t NavModule::GetTargetWaypoint(gps_waypoints_t& target_waypoint)
{
    target_waypoint.lat = target.lat; 
    target_waypoint.lon = target.lon; 
    return waypoint_index; 
}


// Update coordinate radius 
void NavModule::SetCoordinateRadius(int32_t coord_radius)
{
    coordinate_radius = coord_radius; 
}

//=======================================================================================


//=======================================================================================
// Module functions 

// Target waypoint update 
void NavModule::SetTargetLocation(const gps_waypoints_t& waypoint)
{
    target.lat = waypoint.lat; 
    target.lon = waypoint.lon; 
}

//=======================================================================================
