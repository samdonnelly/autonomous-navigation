/**
 * @file hardware.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Vehicle hardware interface 
 * 
 * @details Vehicle hardware functions are not defined by the autopilot. They should be 
 *          defined within the project using this autopilot library so the project can 
 *          add a hardware specific interface. 
 * 
 * @version 0.1
 * @date 2025-02-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _HARDWARE_H_ 
#define _HARDWARE_H_ 

//=======================================================================================
// Includes 

#include "system_tools.h" 

//=======================================================================================

//=======================================================================================
// Classes 

class VehicleHardware 
{
public:   // Public members 

    // These store the data provided by the hardware. The hardware can directly populate 
    // them and the vehicle system can directly access them. 

    struct DataReady 
    {
        uint8_t telemetry_ready : 1; 
    }
    data_ready; 

    // Telemetry 
    uint8_t *telemetry_buff; 
    uint16_t telemetry_data_size; 

public:   // Public methods 

    // Hardware setup 
    void HardwareSetup(void); 

    // GPS 
    void GPS_Read(void); 
    void GPS_Get(void); 

    // Compass 
    void CompassRead(void); 

    // IMU 
    void IMU_Read(void); 

    // Telemetry 
    void TelemetryRead(void); 
    void TelemetryWrite(void); 

    // RC 
    void RC_Read(void); 

    // Memory 
    void MemoryRead(void); 
    void MemoryWrite(void); 

    // Rangefinder 
    void RangefinderRead(void); 
}; 

//=======================================================================================

#endif   // _HARDWARE_H_ 
