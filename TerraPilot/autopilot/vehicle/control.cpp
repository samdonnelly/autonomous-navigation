/**
 * @file control.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle control 
 * 
 * @version 0.1
 * @date 2025-04-10
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Initialization 

// Constructor 
VehicleControl::VehicleControl()
{
    // 
}

//=======================================================================================


//=======================================================================================
// Data decoding 

// Data decode 
void VehicleControl::DataDecode(Vehicle &vehicle)
{
    if (vehicle.hardware.data_ready.rc_ready == FLAG_SET)
    {
        vehicle.hardware.data_ready.rc_ready = FLAG_CLEAR; 

        // Get a copy of the data so we don't have to hold the comms mutex throughout the 
        // whole decoding process. 
        xSemaphoreTake(vehicle.comms_mutex, portMAX_DELAY); 
        // vehicle.hardware.TelemetryGet(data_in_size, data_in_buff); 
        xSemaphoreGive(vehicle.comms_mutex); 
    }
}

//=======================================================================================
