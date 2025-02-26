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
public:   // Friends 

    // // Modules 
    // friend class BoatNav; 
    // friend class BoatRadio; 
    // friend class BoatRC; 

    // // For unit testing only. Do not use anywhere else. 
    // friend class BoatUTest; 

private:   // Private members 

    //==================================================
    // Main thread 
    
    // Threads info 
    // ThreadEventData main_event_info; 

    // States 
    enum class MainStates {
        INIT_STATE,      // Initialization state 
        STANDBY_STATE,   // Standby/Ready state 
        AUTO_STATE,      // Autonomous waypoint navigation state 
        MANUAL_STATE,    // Manual/remote control (RC) state 
        HOME_STATE,      // Return to home location state 
        LOITER_STATE,    // Loiter in one location state 
        FOLLOW_STATE,    // Follow a target state 
        LAUNCH_STATE,    // Launch from dock or trailer state 
        DOCK_STATE,      // Navigate to dock or trailer state 
        LOW_PWR_STATE,   // Low power state 
        FAULT_STATE,     // Fault state 
        RESET_STATE,     // Reset state 
        NUM_STATES       // Not a state, just the total number of states 
    } main_state; 

    // Events 
    enum class MainEvents : uint8_t {
        NO_EVENT, 
        INIT, 
        RADIO_CHECK, 
        NAV_HEADING_CALC, 
        NAV_LOCATION_CALC, 
        RADIO_CONNECTION 
    } main_event; 

    // Flags 
    struct MainFlags 
    {
        // System flags 
        uint8_t state_entry : 1; 
        uint8_t state_exit  : 1; 

        // State flags 
        uint8_t init_state    : 1;   // Initialization state 
        uint8_t standby_state : 1;   // Standby/Ready state 
        uint8_t auto_state    : 1;   // Autonomous waypoint navigation state 
        uint8_t manual_state  : 1;   // Manual/remote control (RC) state 
        uint8_t home_state    : 1;   // Return to home location state 
        uint8_t loiter_state  : 1;   // Loiter in one location state 
        uint8_t follow_state  : 1;   // Follow a target state 
        uint8_t launch_state  : 1;   // Launch from dock or trailer state 
        uint8_t dock_state    : 1;   // Navigate to dock or trailer state 
        uint8_t low_pwr_state : 1;   // Low power state 
        uint8_t fault_state   : 1;   // Fault state 
        uint8_t reset_state   : 1;   // Reset state 
    }
    main_flags; 
    
    //==================================================

    //==================================================
    // Communication thread 

    // Threads info 
    // ThreadEventData comms_event_info; 

    // Events 
    enum class CommsEvents : uint8_t {
        NO_EVENT, 
        LED_STROBE, 
        LED_STROBE_OFF, 
        LED_WRITE, 
        RADIO_READ, 
        RADIO_SEND, 
        NAV_HEADING_UPDATE, 
        NAV_LOCATION_UPDATE 
    } comms_event; 

    //==================================================

    //==================================================
    // Software timers thread 

    TimerHandle_t periodic_timer_100ms; 
    TimerHandle_t periodic_timer_1s; 

    //==================================================

    //==================================================
    // Thread synchronization 

    // General communication thread mutex 
    SemaphoreHandle_t comms_mutex; 

    //==================================================

    //==================================================
    // System data 

    // uint16_t adc_buff[BOAT_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 

    // // Modules 
    // WS2812_Controller leds; 
    // BoatNav navigation; 
    // BoatRadio radio; 
    // BoatRC rc; 
    
    //==================================================

private:   // Private member functions 

    // State function pointer 
    typedef void (*state_func_ptr)(Boat& data, Event event); 

    //==================================================
    // Main thread 

    // Dispatch function 
    static void BoatMainDispatch(Event event); 

    // State functions 
    static void MainInitState(Boat& data, Event event); 
    static void MainStandbyState(Boat& data, Event event); 
    static void MainAutoState(Boat& data, Event event); 
    static void MainManualState(Boat& data, Event event); 
    static void MainHomeState(Boat& data, Event event); 
    static void MainLoiterState(Boat& data, Event event); 
    static void MainFollowState(Boat& data, Event event); 
    static void MainLaunchState(Boat& data, Event event); 
    static void MainDockState(Boat& data, Event event); 
    static void MainLowPwrState(Boat& data, Event event); 
    static void MainFaultState(Boat& data, Event event); 
    static void MainResetState(Boat& data, Event event); 

    // State table 
    const state_func_ptr main_state_table[(uint8_t)MainStates::NUM_STATES] = 
    {
        &MainInitState, 
        &MainStandbyState, 
        &MainAutoState, 
        &MainManualState, 
        &MainHomeState, 
        &MainLoiterState, 
        &MainFollowState, 
        &MainLaunchState, 
        &MainDockState, 
        &MainLowPwrState, 
        &MainFaultState, 
        &MainResetState 
    }; 

    // // State entry/exit functions 
    // void MainInitStateEntry(void); 
    // void MainInitStateExit(void); 
    // void MainStandbyStateEntry(void); 
    // void MainStandbyStateExit(void); 
    // void MainAutoStateEntry(void); 
    // void MainAutoStateExit(void); 
    // void MainManualStateEntry(void); 
    // void MainManualStateExit(void); 
    // void MainLowPwrStateEntry(void); 
    // void MainLowPwrStateExit(void); 
    // void MainFaultStateEntry(void); 
    // void MainFaultStateExit(void); 
    // void MainResetStateEntry(void); 
    // void MainResetStateExit(void); 

    // Helper functions 
    void MainEventQueue(Event event); 
    void MainStateChange(void); 

    //==================================================

    //==================================================
    // Comms thread 

    // Dispatch function 
    static void BoatCommsDispatch(Event event); 

    // Helper functions 
    void CommsEventQueue(Event event); 

    //==================================================

    //==================================================
    // Software timer thread 

    // Callback function(s) 
    static void TimerCallback100ms(TimerHandle_t xTimer); 
    static void TimerCallback1s(TimerHandle_t xTimer); 

    //==================================================

    //==================================================
    // LED module 

    // void LEDStrobeUpdate(uint32_t led_colour); 
    // void LEDStrobeOff(void); 
    // void LEDUpdate(
    //     uint32_t starbird_led_colour, 
    //     uint32_t port_led_colour); 

    //==================================================

public:   // Public member functions 

    // Constructor(s) 
    Boat(); 

    // Destructor 
    ~Boat() {} 

    // // Setup 
    // void BoatSetup(void); 
}; 

extern Boat boat; 

//=======================================================================================

#endif   // _BOAT_H_ 
