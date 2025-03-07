/**
 * @file boat.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat interface 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _BOAT_H_ 
#define _BOAT_H_ 

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Classes 

class Boat : public Vehicle 
{
private:   // Private members 

    // Main thread states - The modes from ArduPilots Rover flightmodes are inserted 
    // here. The numbers for those modes are maintained so that this vehicle can be used 
    // with Mission Planner. Not all of these modes are implemented and the functionality 
    // is not necessarily the same as ArduRover. Additional modes for this system are 
    // also added. 
    // See: https://ardupilot.org/rover/docs/parameters.html#mode1 
    enum class MainStates : uint8_t {
        // ArduRover modes/states 
        MANUAL_STATE,      // 0: Manual/remote control (RC) 
        ACRO_STATE,        // 1: Acro 
        UNUSED1_STATE,     // 2: Unused 1 
        STEERING_STATE,    // 3: Steering 
        HOLD_STATE,        // 4: Hold 
        LOITER_STATE,      // 5: Loiter in one location 
        FOLLOW_STATE,      // 6: Follow a target 
        SIMPLE_STATE,      // 7: Simple 
        DOCK_STATE,        // 8: Navigate to dock or trailer 
        CIRCLE_STATE,      // 9: Circle 
        AUTO_STATE,        // 10: Autonomous waypoint navigation 
        RTL_STATE,         // 11: RTL (Return to Launch) 
        SMART_RTL_STATE,   // 12: Smart RTL (return using known path) 
        UNUSED2_STATE,     // 13: Unused 2 
        UNUSED3_STATE,     // 14: Unused 3 
        GUIDED_STATE,      // 15: Guided 
        INIT_STATE,        // 16: Initialization - defined in the code only 
        // Custom states 
        FAULT_STATE,       // 17: Fault 
        RESET_STATE,       // 18: Reset 
        NUM_STATES         // Not a state, just the total number of states 
    } main_state; 

    // Main thread state flags 
    union MainStateFlags 
    {
        struct 
        {
            uint32_t init_state      : 1;   // Initialization state 
            uint32_t hold_state      : 1;   // Standby/Ready state 
            uint32_t manual_state    : 1;   // Manual/remote control (RC) state 
            uint32_t acro_state      : 1;   // Acro state 
            uint32_t steering_state  : 1;   // Steering state 
            uint32_t loiter_state    : 1;   // Loiter in one location state 
            uint32_t follow_state    : 1;   // Follow a target state 
            uint32_t simple_state    : 1;   // Simple state 
            uint32_t dock_state      : 1;   // Navigate to dock or trailer state 
            uint32_t circle_state    : 1;   // Circle state 
            uint32_t auto_state      : 1;   // Autonomous waypoint navigation state 
            uint32_t rtl_state       : 1;   // Return to home location state 
            uint32_t smart_rtl_state : 1;   // Return to home location on known path state 
            uint32_t guided_state    : 1;   // Guided state 
            uint32_t fault_state     : 1;   // Fault state 
            uint32_t reset_state     : 1;   // Reset state 
        }; 
        uint32_t flags; 
    }
    main_state_flags; 

private:   // Private methods 

    // Boat setup code. Naming and override allow for the Vehicle class to call this 
    // during generic vehicle setup. 
    void VehicleSetup(void) override; 

    // State function pointer 
    typedef void (*state_func_ptr)(Boat& data, Event event); 

    // Dispatch and callback functions 
    static void MainDispatch(Event event); 
    static void CommsDispatch(Event event); 
    static void TimerCallback100ms(TimerHandle_t xTimer); 
    static void TimerCallback1s(TimerHandle_t xTimer); 

    // Main thread state functions 
    static void MainInitState(Boat& data, Event event); 
    static void MainHoldState(Boat& data, Event event); 
    static void MainManualState(Boat& data, Event event); 
    static void MainAcroState(Boat& data, Event event); 
    static void MainSteeringState(Boat& data, Event event); 
    static void MainLoiterState(Boat& data, Event event); 
    static void MainFollowState(Boat& data, Event event); 
    static void MainSimpleState(Boat& data, Event event); 
    static void MainDockState(Boat& data, Event event); 
    static void MainCircleState(Boat& data, Event event); 
    static void MainAutoState(Boat& data, Event event); 
    static void MainRTLState(Boat& data, Event event); 
    static void MainSmartRTLState(Boat& data, Event event); 
    static void MainGuidedState(Boat& data, Event event); 
    static void MainFaultState(Boat& data, Event event); 
    static void MainResetState(Boat& data, Event event); 

    // Main thread state table 
    const state_func_ptr main_state_table[(uint8_t)MainStates::NUM_STATES] = 
    {
        &MainManualState,    // 0: Manual/remote control (RC) 
        &MainAcroState,      // 1: Acro 
        nullptr,             // 2: Unused 1 
        &MainSteeringState,  // 3: Steering 
        &MainHoldState,      // 4: Hold 
        &MainLoiterState,    // 5: Loiter in one location 
        &MainFollowState,    // 6: Follow a target 
        &MainSimpleState,    // 7: Simple 
        &MainDockState,      // 8: Navigate to dock or trailer 
        &MainCircleState,    // 9: Circle 
        &MainAutoState,      // 10: Autonomous waypoint navigation 
        &MainRTLState,       // 11: RTL (Return to Launch) 
        &MainSmartRTLState,  // 12: Smart RTL (return using known path) 
        nullptr,             // 13: Unused 2 
        nullptr,             // 14: Unused 3 
        &MainGuidedState,    // 15: Guided 
        &MainInitState,      // 16: Initialization 
        &MainFaultState,     // 17: Fault 
        &MainResetState      // 18: Reset 
    }; 

    // Helper functions 
    void MainStateSelect(uint8_t state) override; 

public:   // Public methods 

    // Constructor(s) 
    Boat(); 

    // Destructor 
    ~Boat() {} 
}; 

extern Boat boat; 

//=======================================================================================

#endif   // _BOAT_H_ 
