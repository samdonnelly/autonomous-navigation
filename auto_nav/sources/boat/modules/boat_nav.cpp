/**
 * @file boat_nav.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat navigation 
 * 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 
#include "gps_coordinates.h" 

//=======================================================================================


//=======================================================================================
// User functions 

// Heading update 
void BoatNav::HeadingUpdate(void)
{
    lsm303agr_status = lsm303agr_m_update(); 
}


// Heading calculation 
void BoatNav::HeadingCalc(void)
{
    // Find the heading error 
    HeadingError(lsm303agr_m_get_heading()); 

    // If navigation status is lost then set the thruster to zero output 

    // Update the thruster output based on the heading error 

    // // Cap the error if needed so the throttle calculation works 
    // if (error_heading > AB_AUTO_MAX_ERROR)
    // {
    //     error_heading = AB_AUTO_MAX_ERROR; 
    // }
    // else if (error_heading < -AB_AUTO_MAX_ERROR)
    // {
    //     error_heading = -AB_AUTO_MAX_ERROR; 
    // }

    // // Calculate the thruster command: throttle = (base throttle) + error*slope 
    // right_thruster = AB_AUTO_BASE_SPEED - error_heading*ESC_MAX_THROTTLE / 
    //                                                 (AB_AUTO_MAX_ERROR + AB_AUTO_MAX_ERROR); 
    // left_thruster = AB_AUTO_BASE_SPEED +  error_heading*ESC_MAX_THROTTLE / 
    //                                                 (AB_AUTO_MAX_ERROR + AB_AUTO_MAX_ERROR); 

    // esc_readytosky_send(DEVICE_ONE, right_thruster); 
    // esc_readytosky_send(DEVICE_TWO, left_thruster); 
}


// Location update 
void BoatNav::LocationUpdate(void)
{
    if (m8q_get_tx_ready())
    {
        m8q_status = m8q_read_data(); 
        navstat = m8q_get_position_navstat_lock(); 
        boat_coordinates.lat = m8q_get_position_lat(); 
        boat_coordinates.lon = m8q_get_position_lon(); 
    }
}


// Location calculation 
void BoatNav::LocationCalc(void)
{
    if (navstat)
    {
        LocationError(boat_coordinates, gps_waypoints); 
    }
    else 
    {
        // Turn thrusters off 

        // esc_readytosky_send(DEVICE_ONE, 0); 
        // esc_readytosky_send(DEVICE_TWO, 0);  
    }
}


// Target waypoint update 
uint8_t BoatNav::TargetUpdate(uint8_t index)
{
    return SetTargetWaypoint(index, gps_waypoints); 
}

//=======================================================================================
