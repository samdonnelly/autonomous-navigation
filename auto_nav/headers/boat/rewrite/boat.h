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

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Notes 
// - Features: (top level - what the user can see/do) 
//   - Choose between throttle control (0-100% ESC/motor throttle) and speed control (GPS 
//     speed setpoint and throttle changes to match while still capping the throttle at 
//     its max). 
//   - Mission modes: continuous (starts mission over once done), single (stops after 
//     hitting the last waypoint), etc. 
// 
// - Requirements: (end goals of the features under the hood) 
//   - Coordinates stored on the SD card (not hard coded) and nothing can happen if there 
//     is no mission on the card. 
// 
// - Functionality: (the way features are implemented under the hood) 
//   - The ability to more easily write and read complicated files/data on the SD card 
//     needs to be added. 
// 
// - Tasks: 
//   - Read devices: (some are run as backgroud threads) 
//     - GPS 
//     - Magnetometer 
//     - RF transceiver 
//     - SD card 
//     - IMU 
//     - Analogs: battery voltages 
//     - Sonar 
//   
//   - Write devices: 
//     - RF transceiver 
//     - ESCs/motors 
//     - SD card 
//     - LEDs 
//     - Bluetooth 
//   
//   - Processes 
//     - Navigation calculations 
//     - Filters (ex. Kalman) 
//     - Log data 
//     - Motor controller 
//     - Idle - called when there is nothing else to run 
// 
// - Scheduling 
//   - Data is read from sensors on a schedule or in intervals. When then main thread 
//     needs the data it then just grabs the formatted version from the driver. 
// 
// - State machine: 
//   - State machine and RTOS can be integrated. 
//   - Event driven + input driven 
//   - Input driven needs to account for changing external inputs while that state 
//     machine is running. 
// 
// - States: 
//   - Autonomous 
//   - Manual 
//   - Loiter 
// 
// - ArduPilot 
//   - Each platform has a 1kHz timer available. This can help time when tasks are run. 
//     If tasks need to run slower then counters are used to accumulate time. Slow tasks 
//     are not called on the timer thread because they would slow it down. 
//   - Callbacks / function pointer are used to define the scheduler. 
//   - An SD card is used to store data. The filesystem within a root directory is as 
//     follows: 
//     - LOGS : flight logs and stored data 
//     - TERRAIN : terrain data 
//     - STRNG_BAK : parameter data is backup up here on every boot 
//     - scripts : LUA scripts 
//=======================================================================================


//=======================================================================================
// Includes 

#include "includes_drivers.h" 
#include "includes_cpp_drivers.h" 
#include "stm32f4xx_it.h" 

// FreeRTOS 
#include "FreeRTOS.h" 
// #include "task.h" 
#include "cmsis_os2.h" 
#include "queue.h" 
// #include "semphr.h" 
#include "timers.h" 

// Modules 
#include "ws2812_controller.h" 

// Testing 
#include "boat_utest.h" 

//=======================================================================================


//=======================================================================================
// Macros 

// Data 
#define BOAT_ADC_BUFF_SIZE 3   // Number of ADCs used 

//=======================================================================================


//=======================================================================================
// Boat object 

class Boat 
{
public: 

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
        STATE_CHANGE, 
        NAV_CALCS, 
        REMOTE_CONTROL 
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
        LED_STROBE 
    } comms_event; 

    //==================================================

    //==================================================
    // Software timers thread 

    TimerHandle_t periodic_timer_100ms; 

    //==================================================

    //==================================================
    // System data 

    uint16_t adc_buff[BOAT_ADC_BUFF_SIZE];     // ADC buffer - battery and PSU voltage 
    WS2812_Controller leds; 
    
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

    // Helper functions 
    void MainEventQueue(Event event); 

    //==================================================

    //==================================================
    // Comms thread 

    // Dispatch function 
    static void BoatCommsDispatch(Event event); 

    //==================================================

    //==================================================
    // Software timer thread 

    // Callback function(s) 
    static void TimerCallback100ms(TimerHandle_t xTimer); 

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

#ifdef __cplusplus
}
#endif

#endif   // _BOAT_H_ 
