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

#include "includes.h"
#include "telemetry.h"
#include "navigation.h"
#include "control.h"
#include "memory.h"
#include "auxiliary.h"

//=======================================================================================


//=======================================================================================
// Classes 

class VehicleHardware 
{
public:

    enum class MemoryStatus : uint32_t
    {
        MEMORY_OK,             // Memory ok - nothing to report 
        MEMORY_FILE_CREATED,   // File had to be created 
        MEMORY_FILE_OPENED,    // Existing file successfully opened 
        MEMORY_EOF,            // End of file reached 
        MEMORY_CAP_ERROR,      // Volume capacity (free space) error 
        MEMORY_ACCESS_ERROR    // Memory device access error 
    };

    union DataReady 
    {
        struct 
        {
            uint32_t imu_ready       : 1; 
            uint32_t gps_ready       : 1; 
            uint32_t rc_ready        : 1; 
            uint32_t telemetry_ready : 1; 
        };
        uint32_t flags; 
    }
    data_ready;

    // Constructor 
    VehicleHardware()
    {
        data_ready.flags = RESET; 
    }

    // Hardware setup 
    void HardwareSetup(void); 

    // Actuators 
    void PropulsionSet(uint16_t throttle_1, uint16_t throttle_2); 
    void SteeringSet(uint16_t roll, uint16_t pitch, uint16_t yaw); 

    // Debug 
    void DebugWrite(void); 
    
    // GPS 
    void GPSRead(void); 
    bool GPSGet(
        VehicleNavigation::Location &location,
        VehicleNavigation::Location &location_uncertainty,
        VehicleNavigation::Velocity &velocity,
        VehicleNavigation::Velocity &velocity_uncertainty); 
    
    // IMU 
    void IMURead(void); 
    void IMUGet(
        VehicleNavigation::Vector<float> &accel, 
        VehicleNavigation::Vector<float> &gyro, 
        VehicleNavigation::Vector<float> &mag); 

    // Memory 
    MemoryStatus MemorySetup(void);
    void MemorySetFileName(const char *file_name, uint16_t file_name_size);
    MemoryStatus MemoryOpenFile(void);
    MemoryStatus MemoryCloseFile(void);
    MemoryStatus MemoryRead(void);
    void MemoryGetData(char *data_buff, uint16_t data_buff_size);
    void MemorySetData(const char *data_buff, uint16_t data_buff_size);
    MemoryStatus MemoryWrite(void);

    // RC 
    // Serial protocol is used for remote control which means the hardware must read and 
    // write data in the form of channels using a serial protocol (ex. IBUS or SBUS). 
    void RCRead(void); 
    void RCGet(VehicleControl::ChannelFunctions &channels); 

    // Telemetry 
    // MAVLink protocol is used for telemetry which means the hardware must provide 
    // data in the form of MAVLink messages but also accept and send MAVLink messages. 
    void TelemetryRead(void); 
    void TelemetryGet(uint16_t &size, uint8_t *buffer); 
    void TelemetrySet(uint16_t &size, uint8_t *buffer); 
    void TelemetryWrite(void);
}; 

//=======================================================================================

#endif   // _HARDWARE_H_ 
