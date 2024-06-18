/**
 * @file boat.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat interface 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _BOAT_H_
#define _BOAT_H_ 

//=======================================================================================
// Includes 

// System 
#include "includes_drivers.h" 
#include "includes_cpp_drivers.h" 
#include "stm32f4xx_it.h" 

// FreeRTOS 
#include "FreeRTOS.h" 
#include "cmsis_os2.h" 
#include "queue.h" 
#include "semphr.h" 
#include "timers.h" 

// Modules 
#include "boat_nav.h" 
#include "boat_radio.h" 
#include "boat_rc.h" 

// Testing 
#include "boat_utest.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define BOAT_ADC_BUFF_SIZE 3   // Number of ADCs used 

//=======================================================================================


//=======================================================================================
// Boat object 

class Boat 
{
public:   // Friends 

    // Modules 
    friend class BoatNav; 
    friend class BoatRadio; 
    friend class BoatRC; 

    // For unit testing only. Do not use anywhere else. 
    friend class BoatUTest; 

private:   // Private members 

    //==================================================
    // Main thread 
    
    // Threads info 
    ThreadEventData main_event_info; 

    // States 
    enum class MainStates {
        INIT_STATE, 
        STANDBY_STATE, 
        AUTO_STATE, 
        MANUAL_STATE, 
        LOW_PWR_STATE, 
        FAULT_STATE, 
        RESET_STATE, 
        NUM_STATES 
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
        uint8_t init_state    : 1; 
        uint8_t standby_state : 1; 
        uint8_t auto_state    : 1; 
        uint8_t manual_state  : 1; 
        uint8_t low_pwr_state : 1; 
        uint8_t fault_state   : 1; 
        uint8_t reset_state   : 1; 
    }
    main_flags; 
    
    //==================================================

    //==================================================
    // Communication thread 

    // Threads info 
    ThreadEventData comms_event_info; 

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

    uint16_t adc_buff[BOAT_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 

    // Modules 
    WS2812_Controller leds; 
    BoatNav navigation; 
    BoatRadio radio; 
    BoatRC rc; 
    
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
        &MainLowPwrState, 
        &MainFaultState, 
        &MainResetState 
    }; 

    // State entry/exit functions 
    void MainInitStateEntry(void); 
    void MainInitStateExit(void); 
    void MainStandbyStateEntry(void); 
    void MainStandbyStateExit(void); 
    void MainAutoStateEntry(void); 
    void MainAutoStateExit(void); 
    void MainManualStateEntry(void); 
    void MainManualStateExit(void); 
    void MainLowPwrStateEntry(void); 
    void MainLowPwrStateExit(void); 
    void MainFaultStateEntry(void); 
    void MainFaultStateExit(void); 
    void MainResetStateEntry(void); 
    void MainResetStateExit(void); 

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

    void LEDStrobeUpdate(uint32_t led_colour); 
    void LEDStrobeOff(void); 
    void LEDUpdate(
        uint32_t starbird_led_colour, 
        uint32_t port_led_colour); 

    //==================================================

public:   // Public member functions 

    // Constructor(s) 
    Boat(); 

    // Destructor 
    ~Boat() {} 

    // Setup 
    void BoatSetup(void); 
}; 

extern Boat boat; 

//=======================================================================================

#endif   // _BOAT_H_ 
