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

    // General communication thread mutex grab 
    xSemaphoreTake(boat.comms_mutex, portMAX_DELAY); 

    switch (boat.comms_event)
    {
        case CommsEvents::LED_STROBE: 
            boat.leds.Strobe(); 
            break; 

        case CommsEvents::LED_STROBE_OFF: 
            boat.leds.StrobeOff(); 
            break; 

        case CommsEvents::LED_WRITE: 
            boat.leds.LEDWrite(); 
            break; 

        case CommsEvents::RADIO_READ: 
            boat.radio.CommandRead(boat); 
            break; 

        case CommsEvents::RADIO_SEND: 
            boat.radio.CommandSend(); 
            break; 

        case CommsEvents::NAV_HEADING_UPDATE: 
            boat.navigation.HeadingUpdate(); 
            break; 

        case CommsEvents::NAV_LOCATION_UPDATE: 
            boat.navigation.LocationUpdate(); 
            break; 

        default: 
            boat.comms_event = CommsEvents::NO_EVENT; 
            break; 
    }

    // General communication thread mutex release 
    xSemaphoreGive(boat.comms_mutex); 
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Queue an event for the comms thread 
void Boat::CommsEventQueue(Event event)
{
    comms_event_info.event = event; 
    xQueueSend(comms_event_info.ThreadEventQueue, (void *)&comms_event_info.event, 0); 
}

//=======================================================================================
