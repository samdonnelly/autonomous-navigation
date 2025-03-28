/**
 * @file hardware_mock.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Hardware mock 
 * 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "hardware_mock.h" 

//=======================================================================================


//=======================================================================================
// Mock data 

HardwareMock hardware_mock; 

//=======================================================================================


//=======================================================================================
// Library functions 

//==================================================
// Initialization 

void VehicleHardware::HardwareSetup(void)
{
    // 
}

//==================================================

//==================================================
// Telemetry 

uint8_t VehicleHardware::TelemetryRead(void)
{
    return true; 
}


void VehicleHardware::TelemetryGet(uint16_t &size, uint8_t *buffer)
{
    size = hardware_mock.telemetry_in_size; 
    memcpy((void *)buffer, (void *)hardware_mock.telemetry_in_buff, size); 

    hardware_mock.TelmetryInReset(); 
}


void VehicleHardware::TelemetrySet(uint16_t &size, uint8_t *buffer)
{
    hardware_mock.telemetry_out_size = size; 
}


void VehicleHardware::TelemetryWrite(void)
{
    // 
}

//==================================================

//=======================================================================================


//=======================================================================================
// Mock functions 

void HardwareMock::HardwareMockInit(void)
{
    telemetry_out_size = RESET; 
    TelmetryInReset(); 
}


void HardwareMock::TelemetryInAppend(uint16_t size, uint8_t *buffer)
{
    memcpy((void *)(telemetry_in_buff + telemetry_in_size), (void *)buffer, size); 
    telemetry_in_size += size; 
}


void HardwareMock::TelmetryInReset(void)
{
    telemetry_in_size = RESET; 
    memset((void *)telemetry_in_buff, RESET, sizeof(telemetry_in_buff)); 
}


uint16_t HardwareMock::TelemetryOutGetSize(void)
{
    return telemetry_out_size; 
}

//=======================================================================================
