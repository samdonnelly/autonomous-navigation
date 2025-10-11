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
    boat.CommsEventQueue(static_cast<Event>(CommsEvents::TELEMETRY_READ));
    boat.MainEventQueue(static_cast<Event>(MainEvents::TELEMETRY_DECODE));

    // Check for incoming RC (transmitter/receiver) data 
    boat.CommsEventQueue(static_cast<Event>(CommsEvents::RC_READ));
    boat.MainEventQueue(static_cast<Event>(MainEvents::RC_UPDATE));

    // Keep the current orientation up to date 
    boat.CommsEventQueue(static_cast<Event>(CommsEvents::IMU_READ));
    boat.MainEventQueue(static_cast<Event>(MainEvents::IMU_UPDATE));

    // State events 
    if (boat.main_state == MainStates::MANUAL_STATE)
    {
        boat.MainEventQueue(static_cast<Event>(MainEvents::REMOTE_CONTROL));
    }

    // Conditional events 
    if (*parameters[VehicleMemory::LOG_ENABLE].value != 0.0f)
    {
        boat.MainEventQueue(static_cast<Event>(MainEvents::LOG_DATA));
    }
}


// 100ms timer 
void Boat::TimerCallback100ms(void *timer_arg)
{
    // State events 
    if (boat.main_state == MainStates::AUTO_STATE)
    {
        boat.MainEventQueue(static_cast<Event>(MainEvents::COURSE_CORRECT));
    }

#if VH_DEBUG_OUTPUT 

    // Output data for debugging related purposes 
    boat.CommsEventQueue(static_cast<Event>(CommsEvents::DEBUG_WRITE));
    
#endif   // VH_DEBUG_OUTPUT 
}


// 250ms timer 
void Boat::TimerCallback250ms(void *timer_arg)
{
    // Check for telemetry messages to send 
    boat.MainEventQueue(static_cast<Event>(MainEvents::TELEMETRY_ENCODE));
}


// 1s timer 
void Boat::TimerCallback1s(void *timer_arg)
{
    // Keep the current location up to date 
    boat.CommsEventQueue(static_cast<Event>(CommsEvents::GPS_READ));
    boat.MainEventQueue(static_cast<Event>(MainEvents::GPS_UPDATE));

    // State events 
    if (boat.main_state == MainStates::AUTO_STATE)
    {
        boat.MainEventQueue(static_cast<Event>(MainEvents::TARGET_ASSESS));
    }
}

//=======================================================================================
