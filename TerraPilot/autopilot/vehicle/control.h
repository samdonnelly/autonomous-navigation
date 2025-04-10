/**
 * @file control.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle control interface 
 * 
 * @version 0.1
 * @date 2025-03-14
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _CONTROL_H_ 
#define _CONTROL_H_ 

//=======================================================================================
// Includes 

#include "includes.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle; 


class VehicleControl 
{
public:   // public types 

    struct ChannelFunctions 
    {
        uint16_t throttle, roll, pitch, yaw; 
        uint16_t mode_control, mode; 
    }; 

public:   // public methods 

    // Constructor 
    VehicleControl(); 

    // Packet handling 
    void DataDecode(Vehicle &vehicle); 
}; 

//=======================================================================================

#endif   // _CONTROL_H_ 
