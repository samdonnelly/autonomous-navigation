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
        case CommsEvents::GPS_READ: 
            boat.hardware.GPSRead(); 
            break; 

        case CommsEvents::IMU_READ: 
            boat.hardware.IMURead(); 
            break; 

        case CommsEvents::RC_READ: 
            boat.hardware.RCRead(); 
            break; 

        case CommsEvents::TELEMETRY_READ: 
            boat.hardware.TelemetryRead(); 
            break; 

        case CommsEvents::TELEMETRY_WRITE: 
            boat.hardware.TelemetryWrite(); 
            osSemaphoreRelease(boat.telemetry_out_semaphore); 
            break; 

        default: 
            boat.comms_event = CommsEvents::NO_EVENT; 
            break; 
    }

    // General communication thread mutex release 
    xSemaphoreGive(boat.comms_mutex); 
}

//=======================================================================================
