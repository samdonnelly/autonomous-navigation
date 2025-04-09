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

    periodic_timer_100ms.callback = TimerCallback100ms; 
    periodic_timer_250ms.callback = TimerCallback250ms; 
    periodic_timer_1s.callback = TimerCallback1s; 
}

//=======================================================================================
