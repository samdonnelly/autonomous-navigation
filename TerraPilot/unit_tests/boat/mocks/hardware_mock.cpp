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
// Actuators 

void VehicleHardware::PropulsionSet(
    uint16_t throttle_1, 
    uint16_t throttle_2)
{
    hardware_mock.ThrottleSepointCopy(throttle_1, throttle_2); 
}


void VehicleHardware::SteeringSet(
    uint16_t roll, 
    uint16_t pitch, 
    uint16_t yaw)
{
    hardware_mock.SteeringSepointCopy(roll, pitch, yaw); 
}

//==================================================

//==================================================
// GPS 

void VehicleHardware::GPSRead(void)
{
    // 
}


bool VehicleHardware::GPSGet(VehicleNavigation::Location &location)
{
    return false; 
}

//==================================================

//==================================================
// RC 

void VehicleHardware::RCRead(void)
{
    // 
}


void VehicleHardware::RCGet(VehicleControl::ChannelFunctions &channels)
{
    // 
}

//==================================================

//==================================================
// Telemetry 

void VehicleHardware::TelemetryRead(void)
{
    // 
}


void VehicleHardware::TelemetryGet(uint16_t &size, uint8_t *buffer)
{
    // 
}


void VehicleHardware::TelemetrySet(uint16_t &size, uint8_t *buffer)
{
    // 
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
    throttle_setpoint_index = RESET; 
    orientation_setpoint_index = RESET; 
}


void HardwareMock::ThrottleSepointCopy(
    uint16_t throttle_1, 
    uint16_t throttle_2)
{
    if (throttle_setpoint_index < control_setpoints.size())
    {
        control_setpoints[throttle_setpoint_index].throttle_1 = throttle_1; 
        control_setpoints[throttle_setpoint_index].throttle_2 = throttle_2; 

        throttle_setpoint_index++; 
    }
}


void HardwareMock::SteeringSepointCopy(
    uint16_t roll, 
    uint16_t pitch, 
    uint16_t yaw)
{
    if (orientation_setpoint_index < control_setpoints.size())
    {
        control_setpoints[orientation_setpoint_index].roll = roll; 
        control_setpoints[orientation_setpoint_index].pitch = pitch; 
        control_setpoints[orientation_setpoint_index].yaw = yaw; 

        orientation_setpoint_index++; 
    }
}


void HardwareMock::ControlSetpointGet(
    std::array<ControlSetpoints, HW_MOCK_CONTROL_SETPOINTS> &setpoints)
{
    for (uint8_t i = RESET; i < setpoints.size(); i++)
    {
        setpoints[i] = control_setpoints[i]; 
    }
}

//=======================================================================================
