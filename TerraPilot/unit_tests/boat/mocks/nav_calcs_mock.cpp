/**
 * @file nav_calcs_mock.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief 
 * 
 * @version 0.1
 * @date 2024-06-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "nav_calcs.h" 

//=======================================================================================


//=======================================================================================
// Mock implementations 

// Constructor - Default 
nav_calculations::nav_calculations() {}

// Constructor - Specify coordinate filter gain 
nav_calculations::nav_calculations(double coordinate_gain) {}

// Constructor - Specify true north correction offset 
nav_calculations::nav_calculations(int16_t tn_offset) {}

// Constructor - Specify coordinate filter gain and true north correction offset 
nav_calculations::nav_calculations(
    double coordinate_gain, 
    int16_t tn_offset) {}

// Destructor 
nav_calculations::~nav_calculations() {}


// Coordinate filter 
void nav_calculations::coordinate_filter(
    gps_waypoints_t new_data, 
    gps_waypoints_t& filtered_data) const
{
    // 
}


// GPS coordinate radius calculation 
int32_t nav_calculations::gps_radius(
    gps_waypoints_t current, 
    gps_waypoints_t target)
{
    return 0; 
}


// GPS heading calculation 
int16_t nav_calculations::gps_heading(
    gps_waypoints_t current, 
    gps_waypoints_t target)
{
    return 0; 
}


// True north heading 
int16_t nav_calculations::true_north_heading(int16_t heading) const
{
    return 0; 
}


// Heading error 
int16_t nav_calculations::heading_error(
    int16_t current_heading, 
    int16_t target_heading)
{
    return 0; 
}


// Set the GPS coordinate low pass filter gain 
void nav_calculations::set_coordinate_lpf_gain(double coordinate_gain)
{
    // 
}


// Set the true north correction offset 
void nav_calculations::set_tn_offset(int16_t tn_offset)
{
    // 
}

//=======================================================================================

