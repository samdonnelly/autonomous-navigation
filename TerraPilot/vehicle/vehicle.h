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
public:

    // Friends :)
    friend class VehicleControl;
    friend class VehicleMemory;
    friend class VehicleNavigation;
    friend class VehicleTelemetry;

    /**
     * @brief Constructor 
     * 
     * @param vehicle_type : vehicle identification type 
     */
    Vehicle(uint8_t vehicle_type);

    /**
     * @brief Destructor 
     */
    ~Vehicle() = default;

    // Project interface 
    void Setup(void);
    void Loop(void);

protected:

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
    osSemaphoreId_t external_memory_semaphore;

    // Main thread events 
    enum class MainEvents : uint8_t {
        NO_EVENT, 
        INIT, 
        COURSE_CORRECT,     // Adjust the vehicle course/orientation 
        GPS_UPDATE,         // Get the new GPS data 
        IMU_UPDATE,         // Get the new IMU data 
        RC_UPDATE,          // Get the new RC data 
        REMOTE_CONTROL,     // Manually control the vehicle 
        TARGET_ASSESS,      // Assess the current mission target 
        TELEMETRY_DECODE,   // Get and decode any new telemetry data 
        TELEMETRY_ENCODE    // Set telemetry data to be sent 
    } main_event;

    // Communication thread events 
    enum class CommsEvents : uint8_t {
        NO_EVENT,
        DEBUG_WRITE,
        GPS_READ,
        IMU_READ,
        MEMORY_SETUP,
        MEMORY_OPEN,
        MEMORY_CLOSE,
        MEMORY_READ,
        MEMORY_WRITE,
        RC_READ,
        TELEMETRY_READ,
        TELEMETRY_WRITE
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

    // Vehicle statuses 
    VehicleHardware::MemoryStatus external_memory_status;

    // Vehicle specific functions 
    virtual void VehicleSetup(void) = 0;
    virtual void MainStateSelect(uint8_t state) = 0;
    virtual void MainStateRCModeMap(uint8_t &mode) = 0;
    virtual void ManualDrive(VehicleControl::ChannelFunctions main_channels) = 0;
    virtual void AutoDrive(float heading_error) = 0;
    virtual void AutoDriveMaxPWMSet(uint16_t max_pwm) = 0;

    // Helper functions 
    void MainEventQueue(Event event);
    void MainCommonEvents(Vehicle::MainEvents &event);
    void MainStateChange(void);
    void MainStateEnter(uint8_t state, uint32_t &flags);
    void MainStateExit(void);
    void CommsEventQueue(Event event);
};

//=======================================================================================

#endif   // _VEHICLE_H_ 
