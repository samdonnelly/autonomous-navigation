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
    boat.MainEventQueue((Event)MainEvents::RC_UPDATE);

    // Keep the current orientation up to date 
    boat.CommsEventQueue((Event)CommsEvents::IMU_READ); 
    boat.MainEventQueue((Event)MainEvents::IMU_UPDATE); 

    // State events 
    if (boat.main_state == MainStates::MANUAL_STATE)
    {
        boat.MainEventQueue((Event)MainEvents::REMOTE_CONTROL);
    }
}


// 100ms timer 
void Boat::TimerCallback100ms(void *timer_arg)
{
    // State events 
    if (boat.main_state == MainStates::AUTO_STATE)
    {
        boat.MainEventQueue((Event)MainEvents::COURSE_CORRECT); 
    }

#if VH_DEBUG_OUTPUT 

    // Output data for debugging related purposes 
    boat.CommsEventQueue((Event)CommsEvents::DEBUG_WRITE); 
    
#endif   // VH_DEBUG_OUTPUT 
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
    boat.CommsEventQueue((Event)CommsEvents::GPS_READ); 
    boat.MainEventQueue((Event)MainEvents::GPS_UPDATE); 

    // State events 
    if (boat.main_state == MainStates::AUTO_STATE)
    {
        boat.MainEventQueue((Event)MainEvents::TARGET_ASSESS); 
    }
}

//=======================================================================================
