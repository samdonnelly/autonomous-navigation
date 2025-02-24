/**
 * @file boat_timers.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat software timers thread 
 * 
 * @version 0.1
 * @date 2024-03-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Callbacks 

// Note that the frequency of these callback is dependent on the tick period for 
// FreeRTOS. 

// 100ms timer 
void Boat::TimerCallback100ms(TimerHandle_t xTimer)
{
    // Update the LED strobe 
    boat.CommsEventQueue((Event)CommsEvents::LED_STROBE); 

    // Check for radio messages 
    boat.CommsEventQueue((Event)CommsEvents::RADIO_READ); 

    // Update the navigation heading when in the auto state 
    if (boat.main_state == MainStates::AUTO_STATE)
    {
        boat.CommsEventQueue((Event)CommsEvents::NAV_HEADING_UPDATE); 
        boat.MainEventQueue((Event)MainEvents::NAV_HEADING_CALC); 
    }
}


// 1s timer 
void Boat::TimerCallback1s(TimerHandle_t xTimer)
{
    // Keep the current location up to date 
    boat.CommsEventQueue((Event)CommsEvents::NAV_LOCATION_UPDATE); 

    // Queue events based on the state 
    if (boat.main_state == MainStates::AUTO_STATE)
    {
        // Perform navigation location calculations when in the auto state 
        boat.MainEventQueue((Event)MainEvents::NAV_LOCATION_CALC); 
    }
    else if (boat.main_state == MainStates::MANUAL_STATE)
    {
        // Check that the radio is still connected when in manual mode 
        boat.MainEventQueue((Event)MainEvents::RADIO_CONNECTION); 
    }
}

//=======================================================================================
