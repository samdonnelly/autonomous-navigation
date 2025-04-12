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
#include "esc_config.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define GPS_RADIUS 50      // Threshold distance from location to target (m*10) 
#define GPS_LPF_GAIN 0.5   // Coordinate update low pass filter gain 
#define TN_OFFSET 130      // Offset between true and magnetic north (degrees*10) 

// Thrusters 
#define THRUSTER_BASE_SPEED 50       // Base throttle of each thruster (%) 
#define MAX_HEADING_ERROR 600        // Max heading error (degrees*10) - must be within +/-1800 

//=======================================================================================


//=======================================================================================
// Setup 

// Constructor 
BoatNav::BoatNav()
    : NavModule(GPS_RADIUS, GPS_LPF_GAIN, TN_OFFSET), 
      navstat(CLEAR), 
      m8q_status(M8Q_OK), 
      lsm303agr_status(LSM303AGR_OK) {}

//=======================================================================================


//=======================================================================================
// User functions 

// Load waypoint mission 
void BoatNav::LoadMission(void)
{
    // This function is used to retrieve a GPS waypoint mission to use. Right now the 
    // waypoint mission is hard coded and no loading is required. However, this function 
    // must still be called to set the number of waypoints in the navigation module. The 
    // plan for this function is to load a mission from an SD card, save a local copy of 
    // the mission (size limited) and count the number of waypoints before setting it. 
    // Once implemented, if there is no mission on record then prevent the system from 
    // entering auto mode. The SD card could also potentially store the last targeted 
    // waypoint and start from there but for now it will start from waypoint 0. 

    SetNumWaypoints(NUM_GPS_WAYPOINTS); 
    TargetUpdate(RESET_ZERO); 
}


// Heading update 
void BoatNav::HeadingUpdate(void)
{
    lsm303agr_status = lsm303agr_m_update(); 
}


// Heading calculation 
void BoatNav::HeadingCalc(Boat& boat_nav)
{
    // Calculate the heading using the last read magnetometer data. This is protected to 
    // ensure the heading is not updated mid-calculation. 
    xSemaphoreTake(boat_nav.comms_mutex, portMAX_DELAY); 
    int16_t boat_heading = lsm303agr_m_get_heading(); 
    xSemaphoreGive(boat_nav.comms_mutex); 

    // Find the error between the boat's heading and the coordinate heading 
    heading_error = HeadingError(boat_heading); 

    // 'navstat' is updated immediately after reading from the magnetometer in 
    // 'LocationUpdate' so there's no need to protect it here. 
    // Only the thruster output is dictated by the navigation status so the heading 
    // can be kept up to date. 

    if (navstat)
    {
        // Update the thruster output based on the heading error 

        // Cap the error if needed so the throttle calculation works 
        if (heading_error > MAX_HEADING_ERROR)
        {
            heading_error = MAX_HEADING_ERROR; 
        }
        else if (heading_error < -MAX_HEADING_ERROR)
        {
            heading_error = -MAX_HEADING_ERROR; 
        }

        // Calculate the thruster command: throttle = (base throttle) + error*slope 
        right_thruster = THRUSTER_BASE_SPEED - heading_error*ESC_MAX_THROTTLE / 
                                               (MAX_HEADING_ERROR + MAX_HEADING_ERROR); 
        left_thruster  = THRUSTER_BASE_SPEED + heading_error*ESC_MAX_THROTTLE / 
                                               (MAX_HEADING_ERROR + MAX_HEADING_ERROR); 

        esc_send(DEVICE_ONE, right_thruster); 
        esc_send(DEVICE_TWO, left_thruster); 
    }
    else 
    {
        // If navigation status is lost then set the thruster to zero output 
        ThrustersOff(); 
    }
}


// Location update 
void BoatNav::LocationUpdate(void)
{
    if (m8q_get_tx_ready())
    {
        m8q_status = m8q_read_data(); 
        navstat = m8q_get_position_navstat_lock(); 
    }
}


// Location calculation 
void BoatNav::LocationCalc(Boat& boat_nav)
{
    // 'navstat' is updated immediately after reading from the magnetometer in 
    // 'LocationUpdate' so there's no need to protect it here. 
    // If location is lost by the GPS then location reported will just be 0 which 
    // will greatly throw off the navigation calculations. For this reason, the 
    // location calculation is controlled by the navigation status. 

    if (navstat)
    {
        gps_waypoints_t boat_coordinates; 
        GetCoordinates(boat_coordinates, boat_nav); 
        LocationError(boat_coordinates, gps_waypoints); 
    }
}


// Target waypoint update 
uint8_t BoatNav::TargetUpdate(uint8_t index)
{
    return SetTargetWaypoint(index, gps_waypoints); 
}


// Current location update 
void BoatNav::CurrentUpdate(Boat& boat_nav)
{
    gps_waypoints_t boat_coordinates; 
    GetCoordinates(boat_coordinates, boat_nav); 
    SetCurrentLocation(boat_coordinates); 
}


// Turn thrusters off 
void BoatNav::ThrustersOff(void)
{
    esc_send(DEVICE_ONE, ESC_NO_THRUST); 
    esc_send(DEVICE_TWO, ESC_NO_THRUST); 
}

//=======================================================================================


//=======================================================================================
// Module functions 

// Get the boat's coordinates 
void BoatNav::GetCoordinates(
    gps_waypoints_t& coordinates, 
    Boat& boat_nav)
{
    xSemaphoreTake(boat_nav.comms_mutex, portMAX_DELAY); 
    coordinates.lat = m8q_get_position_lat(); 
    coordinates.lon = m8q_get_position_lon(); 
    xSemaphoreGive(boat_nav.comms_mutex); 
}

//=======================================================================================
