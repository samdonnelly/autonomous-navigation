/**
 * @file boat_nav.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat navigation interface 
 * 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_NAV_H_
#define _BOAT_NAV_H_ 

//=======================================================================================
// Includes 

#include "nav_module.h" 

//=======================================================================================


//=======================================================================================
// Classes 

// Forward declare the Boat class 
class Boat; 

class BoatNav : public NavModule 
{
public:   // Public members 

    uint8_t navstat; 

private:   // Private members 

    int16_t heading_error;               // Error between compass and coordinate heading 

    // Thrusters 
    int16_t right_thruster;              // Right thruster throttle 
    int16_t left_thruster;               // Left thruster throttle 

    M8Q_STATUS m8q_status; 
    LSM303AGR_STATUS lsm303agr_status; 

public:   // Public member functions 

    // Constructor 
    BoatNav(); 

    // Destructor 
    ~BoatNav() {}

    // Load waypoint mission 
    void LoadMission(void); 

    // Heading update 
    void HeadingUpdate(void); 

    // Heading calculation 
    void HeadingCalc(Boat& boat_nav); 

    // Location update 
    void LocationUpdate(void); 

    // Location calculation 
    void LocationCalc(Boat& boat_nav); 

    // Target waypoint update 
    uint8_t TargetUpdate(uint8_t index); 

    // Current location update 
    void CurrentUpdate(Boat& boat_nav); 

    // Turn thrusters off 
    void ThrustersOff(void); 

private:   // Private member functions 

    // Get the boat's coordinates 
    void GetCoordinates(
        gps_waypoints_t& coordinates, 
        Boat& boat_nav); 
}; 

//=======================================================================================

#endif   // _BOAT_NAV_H_ 
