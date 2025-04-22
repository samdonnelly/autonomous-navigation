/**
 * @file vehicle_mock.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Generic vehicle mock 
 * 
 * @version 0.1
 * @date 2025-04-22
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle_mock.h" 

//=======================================================================================


//=======================================================================================
// Mock data 

VehicleMock vehicle_mock; 

//=======================================================================================


//=======================================================================================
// Vehicle functions 

// Constructor 
Vehicle::Vehicle(uint8_t vehicle_type) 
    : main_event(MainEvents::NO_EVENT), 
      comms_event(CommsEvents::NO_EVENT), 
      hardware(), 
      telemetry(vehicle_type), 
      navigation(), 
      control(), 
      memory(), 
      auxiliary()
{
    main_system_flags.state_entry = FLAG_SET; 
    main_system_flags.state_exit = FLAG_CLEAR; 
}


// Vehicle setup 
void Vehicle::Setup(void)
{
    // 
}


// Vehicle loop 
void Vehicle::Loop(void)
{
    switch (vehicle_mock.test_function_index)
    {
        case VehicleMock::MANUAL_DRIVE: 
            for (uint8_t i = RESET; i < vehicle_mock.channels.size(); i++)
            {
                ManualDrive(vehicle_mock.channels[i]); 
            }
            break; 
        
        default: 
            break; 
    }
}


// Queue an event for the main thread 
void Vehicle::MainEventQueue(Event event)
{
    // 
}


// Main thread common events 
void Vehicle::MainCommonEvents(Vehicle::MainEvents &event)
{
    // 
}


// Main thread state change 
void Vehicle::MainStateChange(void)
{
    // 
}


// Main thread state enter 
void Vehicle::MainStateEnter(
    uint8_t state, 
    uint32_t &flags)
{
    // 
}


// Main thread state exit 
void Vehicle::MainStateExit(void)
{
    // 
}


// Queue an event for the comms thread 
void Vehicle::CommsEventQueue(Event event)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Mock functions 

void VehicleMock::TestFunctionIndexSet(TestFunctionIndex index)
{
    test_function_index = index; 
}


void VehicleMock::RCChannelSet(
    std::array<VehicleControl::ChannelFunctions, VEHICLE_MOCK_CHANNEL_BUFF> &rc_channels)
{
    for (uint8_t i = RESET; i < rc_channels.size(); i++)
    {
        channels[i] = rc_channels[i]; 
    }
}

//=======================================================================================
