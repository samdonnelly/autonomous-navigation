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
#include "control.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleHardware 
{
public:   // Public members 

    // These store the data provided by the hardware. The hardware can directly populate 
    // them and the vehicle system can directly access them. 

    union DataReady 
    {
        struct 
        {
            uint32_t telemetry_ready : 1; 
            uint32_t rc_ready        : 1; 
        };
        uint32_t flags; 
    }
    data_ready; 

public:   // Public methods 

    // Constructor 
    VehicleHardware() 
    {
        data_ready.flags = RESET; 
    }

    // Hardware setup 
    void HardwareSetup(void); 

    // // GPS 
    // void GPS_Read(void); 
    // void GPS_Get(void); 

    // // Compass 
    // void CompassRead(void); 

    // // IMU 
    // void IMU_Read(void); 

    // RC 
    // Serial protocol is used for remote control which means the hardware must read and 
    // write data in the form of channels using a serial protocol (ex. IBUS or SBUS). 
    void RCRead(void); 
    void RCGet(VehicleControl::ChannelFunctions &channels); 

    // Telemetry 
    // MAVLink protocol is used for telemetry which means the hardware must provide 
    // data in the form of MAVLink messages but also accept and send MAVLink messages. 
    uint8_t TelemetryRead(void); 
    void TelemetryGet(uint16_t &size, uint8_t *buffer); 
    void TelemetrySet(uint16_t &size, uint8_t *buffer); 
    void TelemetryWrite(void); 

    // // Memory 
    // void MemoryRead(void); 
    // void MemoryWrite(void); 

    // // Rangefinder 
    // void RangefinderRead(void); 
}; 

//=======================================================================================

#endif   // _HARDWARE_H_ 
