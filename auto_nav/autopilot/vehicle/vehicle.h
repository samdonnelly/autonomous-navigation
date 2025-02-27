/**
 * @file vehicle.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Generic vehicle interface 
 * 
 * @details Properties common to all vehicles. 
 * 
 * @version 0.1
 * @date 2025-02-21
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _VEHICLE_H_ 
#define _VEHICLE_H_ 

//=======================================================================================
// Includes 

#include "rtos.h" 
#include "vehicle_mavlink.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle 
{
protected:   // Protected members 

    // Thread info 
    ThreadEventData main_event_info; 
    ThreadEventData comms_event_info; 
    TimerThreadData periodic_timer_100ms; 
    TimerThreadData periodic_timer_1s; 

    // Thread synchronization 
    SemaphoreHandle_t comms_mutex; 

    // Main thread events 
    enum class MainEvents : uint8_t {
        NO_EVENT, 
        INIT, 
        RADIO_CHECK, 
        NAV_HEADING_CALC, 
        NAV_LOCATION_CALC, 
        RADIO_CONNECTION 
    } main_event; 

    // Communication thread events 
    enum class CommsEvents : uint8_t {
        NO_EVENT, 
        LED_STROBE, 
        LED_STROBE_OFF, 
        LED_WRITE, 
        RADIO_READ, 
        RADIO_SEND, 
        NAV_HEADING_UPDATE, 
        NAV_LOCATION_UPDATE, 
        GPS_READ, 
        COMPASS_READ, 
        IMU_READ, 
        TELEMETRY_READ, 
        TELEMETRY_WRITE, 
        RC_READ, 
        MEMORY_READ, 
        MEMORY_WRITE, 
        RANGEFINDER_READ 
    } comms_event; 

    // Features a vehicle has 
    VehicleHardware hardware; 
    VehicleMAVLink mavlink; 

protected:   // Protected methods 

    // Initialization 
    virtual void VehicleSetup(void) = 0; 

    // Helper functions 
    void MainEventQueue(Event event); 
    void CommsEventQueue(Event event); 

public:   // Public methods 

    // Project interface 
    void Setup(void); 
    void Loop(void); 
}; 


class VehicleHardware 
{
    // Vehicle hardware functions are not defined by the autopilot. They should be 
    // defined within the project using this autopilot library so the project can 
    // add a hardware specific interface. 

public:   // Public member functions 
    
    // Hardware setup 
    void HardwareSetup(void *vehicle_data); 

    // GPS 
    void GPS_Read(void *vehicle_data); 
    void GPS_Get(void *vehicle_data); 

    // Compass 
    void CompassRead(void *vehicle_data); 

    // IMU 
    void IMU_Read(void *vehicle_data); 

    // Telemetry 
    void TelemetryRead(void *vehicle_data); 
    void TelemetryWrite(void *vehicle_data); 

    // RC 
    void RC_Read(void *vehicle_data); 

    // Memory 
    void MemoryRead(void *vehicle_data); 
    void MemoryWrite(void *vehicle_data); 

    // Rangefinder 
    void RangefinderRead(void *vehicle_data); 
}; 

//=======================================================================================

#endif   // _VEHICLE_H_ 
