/**
 * @file auto_mode.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat auto mode interface 
 * 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_AUTO_MODE_H_ 
#define _BOAT_AUTO_MODE_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "includes_drivers.h" 
#include "includes_cpp_drivers.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class boat_auto_mode : public nav_calculations 
{
private:   // Private members 

    // Timing 
    TIM_TypeDef *timer_nonblocking;          // Timer used for non-blocking delays 
    tim_compare_t nav_timer;                 // LED output timing info 

    // Nav data 
    int32_t radius;                          // Distance between current and target location 

    // Heading 
    int16_t coordinate_heading;              // Heading between current and desired location 
    int16_t compass_heading;                 // Current compass heading 
    int16_t error_heading;                   // Error between compass and coordinate heading 

    // Thrusters 
    int16_t right_thruster;                  // Right thruster throttle 
    int16_t left_thruster;                   // Left thruster throttle 

public:   // Public members 

    // Nav data 
    gps_waypoints_t current;                 // Current location coordinates 
    gps_waypoints_t target;                  // Desired waypoint coordinates 
    uint8_t waypoint_index;                  // GPS coordinate index 
    uint8_t navstat;                         // Position lock status 

public:   // Public member functions 
    
    // Constructor 
    boat_auto_mode(TIM_TypeDef *timer); 

    // Destructor 
    ~boat_auto_mode(); 

    // Autonomous mode 
    void auto_mode(void); 

    // Autonomous mode exit 
    void auto_mode_exit(void); 
}; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _BOAT_AUTO_MODE_H_ 