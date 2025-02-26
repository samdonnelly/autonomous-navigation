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

// Boat definition 
Boat boat; 

//=======================================================================================


//=======================================================================================
// Setup 

// Constructor 
Boat::Boat(void) {} 


// Boat setup 
void Boat::VehicleSetup(void)
{
    main_event_info.dispatch = MainDispatch; 
    comms_event_info.dispatch = CommsDispatch; 

    periodic_timer_100ms.callback = TimerCallback100ms; 
    periodic_timer_1s.callback = TimerCallback1s; 
}

//=======================================================================================
