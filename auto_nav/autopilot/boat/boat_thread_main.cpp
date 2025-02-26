/**
 * @file boat_thread_main.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat main thread 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

//=======================================================================================


//=======================================================================================
// Dispatch 

// Event loop dispatch function for the main thread 
void Boat::MainDispatch(Event event)
{
    MainStates state = boat.main_state; 

    switch (state)
    {
        case MainStates::INIT_STATE: 
            if (boat.main_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_flags.standby_state)
            {
                state = MainStates::STANDBY_STATE; 
            }
            break; 

        case MainStates::STANDBY_STATE: 
            if (boat.main_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_flags.low_pwr_state)
            {
                state = MainStates::LOW_PWR_STATE; 
            }
            else if (boat.main_flags.auto_state)
            {
                state = MainStates::AUTO_STATE; 
            }
            else if (boat.main_flags.manual_state)
            {
                state = MainStates::MANUAL_STATE; 
            }
            break; 

        case MainStates::AUTO_STATE: 
            if (boat.main_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_flags.low_pwr_state)
            {
                state = MainStates::LOW_PWR_STATE; 
            }
            else if (boat.main_flags.standby_state)
            {
                state = MainStates::STANDBY_STATE; 
            }
            else if (boat.main_flags.manual_state)
            {
                state = MainStates::MANUAL_STATE; 
            }
            break; 

        case MainStates::MANUAL_STATE: 
            if (boat.main_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_flags.low_pwr_state)
            {
                state = MainStates::LOW_PWR_STATE; 
            }
            else if (boat.main_flags.standby_state)
            {
                state = MainStates::STANDBY_STATE; 
            }
            else if (boat.main_flags.auto_state)
            {
                state = MainStates::AUTO_STATE; 
            }
            break; 

        case MainStates::HOME_STATE: 
            break; 

        case MainStates::LOITER_STATE: 
            break; 

        case MainStates::FOLLOW_STATE: 
            break; 

        case MainStates::LAUNCH_STATE: 
            break; 

        case MainStates::DOCK_STATE: 
            break; 

        case MainStates::LOW_PWR_STATE: 
            if (boat.main_flags.standby_state)
            {
                state = MainStates::STANDBY_STATE; 
            }
            else if (boat.main_flags.reset_state)
            {
                state = MainStates::RESET_STATE; 
            }
            break; 

        case MainStates::FAULT_STATE: 
            if (boat.main_flags.reset_state)
            {
                state = MainStates::RESET_STATE; 
            }
            break; 

        case MainStates::RESET_STATE: 
            if (boat.main_flags.init_state)
            {
                state = MainStates::INIT_STATE; 
            }
            break; 
        
        default: 
            break; 
    }

    boat.main_state_table[(uint8_t)state](boat, event); 
    boat.main_state = state; 
}

//=======================================================================================


//=======================================================================================
// Initialization state 

void Boat::MainInitState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 
        data.main_flags.standby_state = FLAG_SET; 

        // Start software timers 
        xTimerStart(data.periodic_timer_100ms.handler, 0); 
        xTimerStart(data.periodic_timer_1s.handler, 0); 

        // // Load a waypoint mission if it exists 
        // navigation.LoadMission(); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::INIT: 
            data.MainStateChange(); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.init_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Standby state 

void Boat::MainStandbyState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 

        // navigation.ThrustersOff(); 
        // LEDStrobeUpdate(ws2812_led_standby_not_ready); 
        // radio.MainStandbyStateCmdEnable(FLAG_SET); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            // data.radio.CommandCheck(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.standby_state = FLAG_CLEAR; 

        // LEDStrobeOff(); 
        // radio.MainStandbyStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// Auto state 

void Boat::MainAutoState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 

        // navigation.CurrentUpdate(boat); 
        // LEDStrobeUpdate(ws2812_led_auto_strobe); 
        // LEDUpdate(ws2812_led_auto_star, ws2812_led_auto_port); 
        // radio.MainAutoStateCmdEnable(FLAG_SET); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            // data.radio.CommandCheck(data); 
            break; 
        
        case MainEvents::NAV_HEADING_CALC: 
            // data.navigation.HeadingCalc(data); 
            break; 
        
        case MainEvents::NAV_LOCATION_CALC: 
            // data.navigation.LocationCalc(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.auto_state = FLAG_CLEAR; 

        // navigation.ThrustersOff(); 
        // LEDStrobeOff(); 
        // LEDUpdate(ws2812_led_off, ws2812_led_off); 
        // radio.MainAutoStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// Manual state 

void Boat::MainManualState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 

        // LEDStrobeUpdate(ws2812_led_manual_strobe); 
        // radio.MainManualStateCmdEnable(FLAG_SET); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            // data.radio.CommandCheck(data); 
            break; 
        
        case MainEvents::RADIO_CONNECTION: 
            // data.rc.RadioConnectionCheck(data.radio.ConnectionStatus()); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.manual_state = FLAG_CLEAR; 

        // rc.ThrustersOff(); 
        // LEDStrobeOff(); 
        // radio.MainManualStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// Home state 

void Boat::MainHomeState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.home_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Loiter state 

void Boat::MainLoiterState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.loiter_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Follow state 

void Boat::MainFollowState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.follow_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Launch state 

void Boat::MainLaunchState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.launch_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Dock state 

void Boat::MainDockState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.dock_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Low Power state 

void Boat::MainLowPwrState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 

        // Stop the software timers 
        xTimerStop(data.periodic_timer_100ms.handler, 0); 
        xTimerStop(data.periodic_timer_1s.handler, 0); 

        // If all the software timers are stopped then there will be no radio checks or 
        // LED updates. 
    
        // LEDStrobeUpdate(ws2812_led_low_pwr); 
        // radio.MainLowPwrStateCmdEnable(FLAG_SET); 
    }
    
    data.main_event = (MainEvents)event; 
    
    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            // data.radio.CommandCheck(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }
    
    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.low_pwr_state = FLAG_CLEAR; 

        // LEDStrobeOff(); 
        // radio.MainLowPwrStateCmdEnable(FLAG_CLEAR); 
    }
}
    
//=======================================================================================


//=======================================================================================
// Fault state 

void Boat::MainFaultState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 

        // radio.MainFaultStateCmdEnable(FLAG_SET); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            // data.radio.CommandCheck(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.fault_state = FLAG_CLEAR; 

        // radio.MainFaultStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// Reset state 

void Boat::MainResetState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = FLAG_CLEAR; 

        // Stop the software timers 
        xTimerStop(data.periodic_timer_100ms.handler, 0); 
        xTimerStop(data.periodic_timer_1s.handler, 0); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = FLAG_CLEAR; 
        data.main_flags.state_entry = FLAG_SET; 
        data.main_flags.reset_state = FLAG_CLEAR; 
    }
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Main thread state change 
void Boat::MainStateChange(void)
{
    main_flags.state_exit = FLAG_SET; 
    MainEventQueue((Event)MainEvents::NO_EVENT); 
}

//=======================================================================================
