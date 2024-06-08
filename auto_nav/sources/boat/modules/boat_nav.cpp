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
    // Read from the magnetometer 
    lsm303agr_m_update(); 
}


// Heading calculation 
void BoatNav::HeadingCalc(void)
{
    // Find the heading error 
    int16_t compass_heading = lsm303agr_m_get_heading(); 
    HeadingError(compass_heading); 

    // If navigation status is lost then set the thruster to zero output 

    // Update the thruster output based on the heading error 
}


// Location update 
void BoatNav::LocationUpdate(void)
{
    // Read from the GPS module 
    if (m8q_get_tx_ready())
    {
        m8q_read_data(); 
    }
}


// Location calculation 
void BoatNav::LocationCalc(void)
{
    // Update the boat's location 

    uint8_t navstat = m8q_get_position_navstat_lock(); 

    if (navstat)
    {
        // Get the M8Q coordinates and pass them to the nav module 

        gps_waypoints_t device_coordinates = 
        {
            .lat = m8q_get_position_lat(), 
            .lon = m8q_get_position_lon() 
        }; 

        LocationError(device_coordinates, gps_waypoints); 
    }
}

//=======================================================================================
