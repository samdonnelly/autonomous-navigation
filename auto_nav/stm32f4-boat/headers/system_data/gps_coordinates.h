/**
 * @file gps_coordinates.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief GPS coordinates file interface 
 *        
 *        Note: the implementation of this header is not tracked because coordinates 
 *              are use case specific. 
 * 
 * @version 0.1
 * @date 2024-03-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _GPS_COORDINATES_H_ 
#define _GPS_COORDINATES_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "hardware_config.h" 
#include "nav_calcs.h"

//=======================================================================================


//=======================================================================================
// Coordinates 

extern const gps_waypoints_t gps_waypoints[NUM_GPS_WAYPOINTS]; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif  // _GPS_COORDINATES_H_
