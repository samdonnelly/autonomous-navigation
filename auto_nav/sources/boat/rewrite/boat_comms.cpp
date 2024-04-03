/**
 * @file boat_comms.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat communication thread 
 * 
 * @version 0.1
 * @date 2024-04-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Dispatch 

// Event loop dispatch function for the comms thread 
void Boat::BoatCommsDispatch(Event event)
{
    boat.comms_event = (CommsEvents)event; 

    switch (boat.comms_event)
    {
        case CommsEvents::LED_STROBE: 
            boat.leds.Strobe(); 
            break; 

        default: 
            break; 
    }
}

//=======================================================================================
