/**
 * @file hardware_mock.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Hardware mock interface 
 * 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _HARDWARE_MOCK_H_ 
#define _HARDWARE_MOCK_H_ 

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Mock data 

class HardwareMock 
{
public: 

    // GPS 
    VehicleNavigation::Location gps; 

    // IMU 
    VehicleNavigation::Vector<int16_t> magnetometer; 

    // Telemetry 
    uint8_t telemetry_in_buff[vs_telemetry_buff]; 
    uint16_t telemetry_in_size; 
    uint16_t telemetry_out_size; 

public: 

    void HardwareMockInit(void); 

    // GPS 
    void GPSSetLocation(VehicleNavigation::Location current_location); 

    // IMU 
    void IMUSetAxisData(VehicleNavigation::Vector<int16_t> mag_axis); 

    // Telemetry 
    void TelemetryInAppend(uint16_t size, uint8_t *buffer); 
    void TelmetryInReset(void); 
    uint16_t TelemetryOutGetSize(void); 
}; 

extern HardwareMock hardware_mock; 

//=======================================================================================

#endif   // _HARDWARE_MOCK_H_ 
