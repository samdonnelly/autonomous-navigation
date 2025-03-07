/**
 * @file boat_thread_comms.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat communications thread 
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
// Dispatch 

// Event loop dispatch function for the comms thread 
void Boat::CommsDispatch(Event event)
{
    boat.comms_event = (CommsEvents)event; 

    // General communication thread mutex grab 
    xSemaphoreTake(boat.comms_mutex, portMAX_DELAY); 

    switch (boat.comms_event)
    {
        // case CommsEvents::LED_STROBE: 
        //     // boat.leds.Strobe(); 
        //     break; 

        // case CommsEvents::LED_STROBE_OFF: 
        //     // boat.leds.StrobeOff(); 
        //     break; 

        // case CommsEvents::LED_WRITE: 
        //     // boat.leds.LEDWrite(); 
        //     break; 

        // case CommsEvents::RADIO_READ: 
        //     // boat.radio.CommandRead(boat); 
        //     break; 

        // case CommsEvents::RADIO_SEND: 
        //     // boat.radio.CommandSend(); 
        //     break; 

        // case CommsEvents::NAV_HEADING_UPDATE: 
        //     // boat.navigation.HeadingUpdate(); 
        //     break; 

        // case CommsEvents::NAV_LOCATION_UPDATE: 
        //     // boat.navigation.LocationUpdate(); 
        //     break; 

        // case CommsEvents::GPS_READ: 
        //     boat.hardware.GPS_Read(); 
        //     break; 

        // case CommsEvents::COMPASS_READ: 
        //     boat.hardware.CompassRead(); 
        //     break; 

        // case CommsEvents::IMU_READ: 
        //     boat.hardware.IMU_Read(); 
        //     break; 

        case CommsEvents::TELEMETRY_READ: 
            boat.hardware.TelemetryRead(); 
            break; 

        case CommsEvents::TELEMETRY_WRITE: 
            boat.hardware.TelemetryWrite(); 
            xSemaphoreGive(boat.telemetry_out_mutex); 
            break; 

        // case CommsEvents::RC_READ: 
        //     boat.hardware.RC_Read(); 
        //     break; 

        // case CommsEvents::MEMORY_READ: 
        //     boat.hardware.MemoryRead(); 
        //     break; 

        // case CommsEvents::MEMORY_WRITE: 
        //     boat.hardware.MemoryWrite(); 
        //     break; 

        // case CommsEvents::RANGEFINDER_READ: 
        //     boat.hardware.RangefinderRead(); 
        //     break; 

        default: 
            boat.comms_event = CommsEvents::NO_EVENT; 
            break; 
    }

    // General communication thread mutex release 
    xSemaphoreGive(boat.comms_mutex); 
}

//=======================================================================================
