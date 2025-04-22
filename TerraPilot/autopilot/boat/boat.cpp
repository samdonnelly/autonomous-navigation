/**
 * @file boat.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Global data 

Boat boat; 

//=======================================================================================


//=======================================================================================
// Setup 

// Constructor 
Boat::Boat() 
    : Vehicle(MAV_TYPE_SURFACE_BOAT), 
      main_state(MainStates::INIT_STATE)
{
    main_state_flags.flags = RESET; 
    main_state_flags.init_state = FLAG_SET; 
}


// Boat setup 
void Boat::VehicleSetup(void)
{
    main_event_info.dispatch = MainDispatch; 
    comms_event_info.dispatch = CommsDispatch; 

    periodic_timer_50ms.callback = TimerCallback50ms; 
    periodic_timer_100ms.callback = TimerCallback100ms; 
    periodic_timer_250ms.callback = TimerCallback250ms; 
    periodic_timer_1s.callback = TimerCallback1s; 
}

//=======================================================================================


//=======================================================================================
// Vehicle Control 

// Manual drive output 
void Boat::ManualDrive(VehicleControl::ChannelFunctions main_channels)
{
#if VS_BOAT_K1 
    
    // With this setup the steering command has to be mapped to a thruster output. 

    uint16_t 
    left_thruster = VehicleControl::PWM_NEUTRAL, 
    right_thruster = VehicleControl::PWM_NEUTRAL; 

    // Map these 
    main_channels.throttle; 
    main_channels.roll; 

    hardware.PropulsionSet(left_thruster, right_thruster); 

#elif VS_BOAT_K2 

    // With this setup the inputs can directly be applied to the motor output and the 
    // unused DOFs set to neutral. 
    hardware.PropulsionSet(main_channels.throttle, VehicleControl::PWM_NEUTRAL); 
    hardware.SteeringSet(main_channels.roll, VehicleControl::PWM_NEUTRAL, VehicleControl::PWM_NEUTRAL); 

#endif 
}


// Autonomous drive output 
void Boat::AutoDrive(int16_t heading_error)
{
#if VS_BOAT_K1 
    // 
#elif VS_BOAT_K2 
#endif 
}

//=======================================================================================
