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
}

//=======================================================================================
