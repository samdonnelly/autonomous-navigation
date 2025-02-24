/**
 * @file vehicle_mavlink.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle MAVLink interface 
 * 
 * @details MAVLink messages and properites common to all vehicles. 
 * 
 * @version 0.1
 * @date 2025-02-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _VEHICLE_MAVLINK_H_ 
#define _VEHICLE_MAVLINK_H_ 

//=======================================================================================
// Includes 

extern "C"
{
    // For C headers without C++ guards 
    #include "common/mavlink.h" 
}

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleMAVLink 
{
public:   // Messages 

    // Incoming 
    mavlink_heartbeat_t heartbeat_msg_gcs; 

    // Outgoing 
    mavlink_heartbeat_t heartbeat_msg;                           // HEARTBEAT 
}; 

//=======================================================================================

#endif   // _VEHICLE_MAVLINK_H_ 
