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

class BoatNav : public NavModule 
{
public:   // Public members 

    uint8_t navstat; 

private:   // Private members 

    gps_waypoints_t boat_coordinates; 

    M8Q_STATUS m8q_status; 
    LSM303AGR_STATUS lsm303agr_status; 

public:   // Public member functions 

    // Constructor 
    BoatNav() 
        : NavModule(100, 0.5, 130), 
          navstat(CLEAR), 
          m8q_status(M8Q_OK), 
          lsm303agr_status(LSM303AGR_OK) 
    {
        boat_coordinates.lat = CLEAR; 
        boat_coordinates.lon = CLEAR; 
    }

    // Destructor 
    ~BoatNav() {}

    // Heading update 
    void HeadingUpdate(void); 

    // Heading calculation 
    void HeadingCalc(void); 

    // Location update 
    void LocationUpdate(void); 

    // Location calculation 
    void LocationCalc(void); 

    // Target waypoint update 
    uint8_t TargetUpdate(uint8_t index); 
}; 

//=======================================================================================

#endif   // _BOAT_NAV_H_ 
