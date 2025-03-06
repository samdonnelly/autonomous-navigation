/**
 * @file vehicle_hardware.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle hardware 
 * 
 * @details This file is added by the project (not the autopilot) to define the vehicle 
 *          hardware functions. These functions provide a hardware specific interface 
 *          that's unique to each project that uses the autopilot. The autopilot 
 *          purposely does not define these because their definition will change 
 *          depending on what the user wants to use. 
 * 
 * @version 0.1
 * @date 2025-02-27
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Include 

#include "hardware.h" 

//=======================================================================================


//=======================================================================================
// Initialization 

void VehicleHardware::HardwareSetup(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// GPS 

// void VehicleHardware::GPS_Read(void)
// {
//     // 
// }


// void VehicleHardware::GPS_Get(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Compass 

// void VehicleHardware::CompassRead(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// IMU 

// void VehicleHardware::IMU_Read(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Telemetry 

void VehicleHardware::TelemetryRead(void)
{
    // Read from radio (if needed) 
    // - With SiK radios over UART I will likely use DMA instead so no need to read here. 
    
    // Set a flag to indicate new data if it's available. 
    // Check interrupt flag status 
    // If true then clear interrupt flag and set telemetry data flag 
    data_ready.telemetry_ready; 
    
    // - We dom't want to process messages here because that is not a time-dependent 
    //   activity. 
}


void VehicleHardware::TelemetryGet(uint16_t &size, uint8_t *buffer)
{
    size;   // Assign size of data received 
    memcpy((void *)buffer, (void *)1, size);   // Copy from hardware to autopilot buffer 
}


void VehicleHardware::TelemetryWrite(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// RC 

// void VehicleHardware::RC_Read(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Memory 

// void VehicleHardware::MemoryRead(void)
// {
//     // 
// }


// void VehicleHardware::MemoryWrite(void)
// {
//     // 
// }

//=======================================================================================


//=======================================================================================
// Rangefinder 

// void VehicleHardware::RangefinderRead(void)
// {
//     // 
// }

//=======================================================================================
