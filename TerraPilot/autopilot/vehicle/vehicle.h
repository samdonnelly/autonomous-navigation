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

#include "includes.h" 

#include "hardware.h" 
#include "telemetry.h" 
#include "navigation.h" 
#include "control.h" 
#include "memory.h" 
#include "auxiliary.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class Vehicle 
{
public:   // Friends 

    friend class VehicleControl; 
    friend class VehicleNavigation; 
    friend class VehicleTelemetry; 

protected:   // Protected members 

    // Thread info 
    ThreadEventData main_event_info; 
    ThreadEventData comms_event_info; 
    TimerThreadData periodic_timer_50ms; 
    TimerThreadData periodic_timer_100ms; 
    TimerThreadData periodic_timer_250ms; 
    TimerThreadData periodic_timer_1s; 

    // Thread synchronization 
    SemaphoreHandle_t comms_mutex; 
    osSemaphoreId_t telemetry_out_semaphore; 

    // Main thread events 
    enum class MainEvents : uint8_t {
        NO_EVENT, 
        INIT, 
        GPS_UPDATE, 
        RC_DECODE, 
        // REMOTE_CONTROL, 
        TELEMETRY_DECODE, 
        TELEMETRY_ENCODE, 
        // NAV_HEADING_CALC, 
        // NAV_LOCATION_CALC 
    } main_event; 

    // Communication thread events 
    enum class CommsEvents : uint8_t {
        NO_EVENT, 
        // LED_STROBE, 
        // LED_STROBE_OFF, 
        // LED_WRITE, 
        // RADIO_READ, 
        // RADIO_SEND, 
        // NAV_HEADING_UPDATE, 
        // NAV_LOCATION_UPDATE, 
        // COMPASS_READ, 
        GPS_READ, 
        // IMU_READ, 
        RC_READ, 
        TELEMETRY_READ, 
        TELEMETRY_WRITE, 
        // MEMORY_READ, 
        // MEMORY_WRITE, 
        // RANGEFINDER_READ 
    } comms_event; 

    // System flags 
    struct SystemFlags 
    {
        uint8_t state_entry : 1; 
        uint8_t state_exit  : 1; 
    }
    main_system_flags; 

    // Features a vehicle has 
    VehicleHardware hardware; 
    VehicleTelemetry telemetry; 
    VehicleNavigation navigation; 
    VehicleControl control; 
    VehicleMemory memory; 
    VehicleAuxiliary auxiliary; 

protected:   // Protected methods 

    // Vehicle specific functions 
    virtual void VehicleSetup(void) = 0; 
    virtual void MainStateSelect(uint8_t state) = 0; 
    virtual void MainStateRCModeMap(uint8_t &mode) = 0; 

    // Helper functions 
    void MainEventQueue(Event event); 
    void MainCommonEvents(Vehicle::MainEvents &event); 
    void MainStateChange(void); 
    void MainStateEnter(uint8_t state, uint32_t &flags); 
    void MainStateExit(void); 
    void CommsEventQueue(Event event); 

public:   // Public methods 

    // Constructor 
    Vehicle(uint8_t vehicle_type); 

    // Project interface 
    void Setup(void); 
    void Loop(void); 
};

//=======================================================================================

#endif   // _VEHICLE_H_ 
