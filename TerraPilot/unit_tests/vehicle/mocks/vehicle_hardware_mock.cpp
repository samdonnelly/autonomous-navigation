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

#include "vehicle_hardware_mock.h" 

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
    // 
}


void VehicleHardware::SteeringSet(
    uint16_t roll, 
    uint16_t pitch, 
    uint16_t yaw)
{
    // 
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
    return true; 
}

//==================================================

//==================================================
// IMU 

void VehicleHardware::IMURead(void)
{
    // 
}


void VehicleHardware::IMUGet(
    VehicleNavigation::Vector<int16_t> &accel, 
    VehicleNavigation::Vector<int16_t> &gyro, 
    VehicleNavigation::Vector<int16_t> &mag, 
    int16_t &heading)
{
    mag.x = hardware_mock.magnetometer.x; 
    mag.y = hardware_mock.magnetometer.y; 
    mag.z = hardware_mock.magnetometer.z; 
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
    // IMU 
    magnetometer.z = RESET; 
    magnetometer.y = RESET; 
    magnetometer.z = RESET; 

    // Telemetry 
    telemetry_out_size = RESET; 
    TelmetryInReset(); 
}


//==================================================
// IMU 

void HardwareMock::IMUSetAxisData(VehicleNavigation::Vector<int16_t> mag_axis)
{
    magnetometer.x = mag_axis.x; 
    magnetometer.y = mag_axis.y; 
    magnetometer.z = mag_axis.z; 
}

//==================================================


//==================================================
// Telemetry 

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

//==================================================

//=======================================================================================
