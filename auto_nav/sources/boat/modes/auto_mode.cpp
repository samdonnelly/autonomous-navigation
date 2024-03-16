/**
 * @file auto_mode.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat auto mode 
 * 
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "auto_mode.h" 
#include "gps_coordinates.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Timing 
#define AB_NAV_UPDATE 100000         // Navigation calculation update period (us) 
#define AB_NAV_COUNTER 10            // update*counter = time between nav calc updates 

// Configuration 
#define AB_COORDINATE_LPF_GAIN 0.5   // Coordinate low pass filter gain 

// Navigation 
#define AB_TN_COR 130                // True North direction correction 
#define AB_WAYPOINT_RAD 100          // Threshold waypoint radius (meters*10) 
#define AB_AUTO_BASE_SPEED 50        // Base throttle of each thruster (%) 
#define AB_AUTO_MAX_ERROR 600        // Max heading error (degrees*10) - must be within +/-1800 

// Thuster 
#define AB_NO_THRUST 0               // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Setup and teardown 

// Constructor 
boat_auto_mode::boat_auto_mode(TIM_TypeDef *timer)
    : nav_calculations(AB_COORDINATE_LPF_GAIN, AB_TN_COR), 
      timer_nonblocking(timer), 
      radius(CLEAR), 
      coordinate_heading(CLEAR), 
      compass_heading(CLEAR), 
      error_heading(CLEAR), 
      right_thruster(AB_NO_THRUST), 
      left_thruster(AB_NO_THRUST), 
      waypoint_index(CLEAR), 
      navstat(CLEAR) 
{
    // Timing 
    memset((void *)&nav_timer, CLEAR, sizeof(nav_timer)); 
    nav_timer.clk_freq = tim_get_pclk_freq(timer); 
    nav_timer.time_start = SET_BIT; 

    // Navigation 
    memset((void *)&current, CLEAR, sizeof(current)); 
    memset((void *)&target, CLEAR, sizeof(target)); 
}


// Destructor 
boat_auto_mode::~boat_auto_mode() {}

//=======================================================================================


//=======================================================================================
// Auto mode functions 

// Autonomous mode 
void boat_auto_mode::auto_mode(void)
{
    static uint8_t nav_period_counter = CLEAR; 

    // Update the navigation calculations 
    if (tim_compare(timer_nonblocking, 
                    nav_timer.clk_freq, 
                    AB_NAV_UPDATE, 
                    &nav_timer.time_cnt_total, 
                    &nav_timer.time_cnt, 
                    &nav_timer.time_start))
    {
        // Update the compass heading, determine the true north heading and find the 
        // error between the current (compass) and desired (GPS) headings. Heading error 
        // is determined here and not with each location update so it's updated faster. 
        lsm303agr_m_update();   // Add status return storage 
        compass_heading = true_north_heading(lsm303agr_m_get_heading()); 
        error_heading = heading_error(compass_heading, coordinate_heading); 

        // Update the GPS information and user navigation info 
        if (nav_period_counter++ >= AB_NAV_COUNTER)
        {
            nav_period_counter = CLEAR; 

            if (navstat)
            {
                // Get the updated location by reading the GPS device coordinates then filtering 
                // the result. 
                gps_waypoints_t device_coordinates = 
                {
                    .lat = m8q_get_position_lat(), 
                    .lon = m8q_get_position_lon() 
                }; 
                coordinate_filter(device_coordinates, current); 

                // Calculate the distance to the target location and the heading needed to get 
                // there. 
                radius = gps_radius(current, target); 
                coordinate_heading = gps_heading(current, target); 

                // Check if the distance to the target is within the threshold. If so, the 
                // target is considered "hit" and we can move to the next target. 
                if (radius < AB_WAYPOINT_RAD)
                {
                    // Adjust waypoint index 
                    if (++waypoint_index >= NUM_GPS_WAYPOINTS)
                    {
                        waypoint_index = CLEAR; 
                    }

                    // Update the target waypoint 
                    target.lat = gps_waypoints[waypoint_index].lat; 
                    target.lon = gps_waypoints[waypoint_index].lon; 
                }
            }
        }

        // Cap the error if needed so the throttle calculation works 
        if (error_heading > AB_AUTO_MAX_ERROR)
        {
            error_heading = AB_AUTO_MAX_ERROR; 
        }
        else if (error_heading < -AB_AUTO_MAX_ERROR)
        {
            error_heading = -AB_AUTO_MAX_ERROR; 
        }

        // Calculate the thruster command: throttle = (base throttle) + error*slope 
        right_thruster = AB_AUTO_BASE_SPEED - error_heading*ESC_MAX_THROTTLE / 
                                                      (AB_AUTO_MAX_ERROR + AB_AUTO_MAX_ERROR); 
        left_thruster = AB_AUTO_BASE_SPEED +  error_heading*ESC_MAX_THROTTLE / 
                                                      (AB_AUTO_MAX_ERROR + AB_AUTO_MAX_ERROR); 

        esc_readytosky_send(DEVICE_ONE, right_thruster); 
        esc_readytosky_send(DEVICE_TWO, left_thruster); 
    }
}


// Autonomous mode exit 
void boat_auto_mode::auto_mode_exit(void)
{
    nav_timer.time_start = SET_BIT; 
    // Resert period counter 

    esc_readytosky_send(DEVICE_ONE, AB_NO_THRUST); 
    esc_readytosky_send(DEVICE_TWO, AB_NO_THRUST); 
}

//=======================================================================================
