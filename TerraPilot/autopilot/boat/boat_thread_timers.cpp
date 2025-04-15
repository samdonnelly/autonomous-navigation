/**
 * @file boat_thread_timers.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat timers thread 
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
// Callbacks 

// 50ms timer 
void Boat::TimerCallback50ms(void *timer_arg)
{
    // Check for incoming telemetry messages 
    boat.CommsEventQueue((Event)CommsEvents::TELEMETRY_READ); 
    boat.MainEventQueue((Event)MainEvents::TELEMETRY_DECODE); 

    // Check for incoming RC (transmitter/receiver) data 
    boat.CommsEventQueue((Event)CommsEvents::RC_READ); 
    boat.MainEventQueue((Event)MainEvents::RC_DECODE); 
}


// 100ms timer 
void Boat::TimerCallback100ms(void *timer_arg)
{
    // Update the LED strobe 
    // boat.CommsEventQueue((Event)CommsEvents::LED_STROBE); 

    // Update the navigation heading when in the auto state 
    // if (boat.main_state == MainStates::AUTO_STATE)
    // {
        // boat.CommsEventQueue((Event)CommsEvents::NAV_HEADING_UPDATE); 
        // boat.MainEventQueue((Event)MainEvents::NAV_HEADING_CALC); 
    // }
}


// 250ms timer 
void Boat::TimerCallback250ms(void *timer_arg)
{
    // Check for telemetry messages to send 
    boat.MainEventQueue((Event)MainEvents::TELEMETRY_ENCODE); 
}


// 1s timer 
void Boat::TimerCallback1s(void *timer_arg)
{
    // Keep the current location up to date 
    // boat.CommsEventQueue((Event)CommsEvents::NAV_LOCATION_UPDATE); 
    boat.CommsEventQueue((Event)CommsEvents::GPS_READ); 
    boat.MainEventQueue((Event)MainEvents::GPS_UPDATE); 

    // // Queue events based on the state 
    // if (boat.main_state == MainStates::AUTO_STATE)
    // {
    //     // Perform navigation location calculations when in the auto state 
    //     boat.MainEventQueue((Event)MainEvents::NAV_LOCATION_CALC); 
    // }
    // else if (boat.main_state == MainStates::MANUAL_STATE)
    // {
    //     // Check that the radio is still connected when in manual mode 
    //     boat.MainEventQueue((Event)MainEvents::RADIO_CONNECTION); 
    // }
}

//=======================================================================================
