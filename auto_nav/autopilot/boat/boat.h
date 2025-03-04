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
        // Custom states 
        INIT_STATE,        // 16: Initialization 
        STANDBY_STATE,     // 17: Standby/Ready 
        LAUNCH_STATE,      // 18: Launch from dock or trailer 
        LOW_PWR_STATE,     // 19: Low power 
        FAULT_STATE,       // 20: Fault 
        RESET_STATE,       // 21: Reset 
        NUM_STATES         // Not a state, just the total number of states 
    } main_state; 

    // Main thread flags 
    struct MainFlags 
    {
        // System flags 
        uint8_t state_entry : 1; 
        uint8_t state_exit  : 1; 

        // State flags 
        uint8_t init_state    : 1;   // Initialization state 
        uint8_t standby_state : 1;   // Standby/Ready state 
        uint8_t manual_state  : 1;   // Manual/remote control (RC) state 
        uint8_t loiter_state  : 1;   // Loiter in one location state 
        uint8_t follow_state  : 1;   // Follow a target state 
        uint8_t launch_state  : 1;   // Launch from dock or trailer state 
        uint8_t dock_state    : 1;   // Navigate to dock or trailer state 
        uint8_t auto_state    : 1;   // Autonomous waypoint navigation state 
        uint8_t rtl_state     : 1;   // Return to home location state 
        uint8_t low_pwr_state : 1;   // Low power state 
        uint8_t fault_state   : 1;   // Fault state 
        uint8_t reset_state   : 1;   // Reset state 
    }
    main_flags; 

    //==================================================
    // System data 

    // uint16_t adc_buff[BOAT_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 

    // // Modules 
    // WS2812_Controller leds; 
    // BoatNav navigation; 
    // BoatRadio radio; 
    // BoatRC rc; 
    
    //==================================================

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
    static void MainStandbyState(Boat& data, Event event); 
    static void MainManualState(Boat& data, Event event); 
    static void MainLoiterState(Boat& data, Event event); 
    static void MainFollowState(Boat& data, Event event); 
    static void MainLaunchState(Boat& data, Event event); 
    static void MainDockState(Boat& data, Event event); 
    static void MainAutoState(Boat& data, Event event); 
    static void MainRTLState(Boat& data, Event event); 
    static void MainLowPwrState(Boat& data, Event event); 
    static void MainFaultState(Boat& data, Event event); 
    static void MainResetState(Boat& data, Event event); 

    // Main thread state table 
    const state_func_ptr main_state_table[(uint8_t)MainStates::NUM_STATES] = 
    {
        &MainManualState,    // 0: Manual/remote control (RC) 
        nullptr,             // 1: Acro 
        nullptr,             // 2: Unused 1 
        nullptr,             // 3: Steering 
        nullptr,             // 4: Hold 
        &MainLoiterState,    // 5: Loiter in one location 
        &MainFollowState,    // 6: Follow a target 
        nullptr,             // 7: Simple 
        &MainDockState,      // 8: Navigate to dock or trailer 
        nullptr,             // 9: Circle 
        &MainAutoState,      // 10: Autonomous waypoint navigation 
        &MainRTLState,       // 11: RTL (Return to Launch) 
        nullptr,             // 12: Smart RTL (return using known path) 
        nullptr,             // 13: Unused 2 
        nullptr,             // 14: Unused 3 
        nullptr,             // 15: Guided 
        &MainInitState,      // 16: Initialization 
        &MainStandbyState,   // 17: Standby/Ready 
        &MainLaunchState,    // 18: Launch from dock or trailer 
        &MainLowPwrState,    // 19: Low power 
        &MainFaultState,     // 20: Fault 
        &MainResetState      // 21: Reset 
        
    }; 

    // Helper functions 
    void MainStateChange(void); 
    void MainStateSelect(uint8_t state); 

    //==================================================
    // LED module 

    // void LEDStrobeUpdate(uint32_t led_colour); 
    // void LEDStrobeOff(void); 
    // void LEDUpdate(
    //     uint32_t starbird_led_colour, 
    //     uint32_t port_led_colour); 

    //==================================================

public:   // Public methods 

    // Constructor(s) 
    Boat(); 

    // Destructor 
    ~Boat() {} 
}; 

extern Boat boat; 

//=======================================================================================

#endif   // _BOAT_H_ 
