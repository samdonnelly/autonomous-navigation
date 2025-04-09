/**
 * @file boat_main.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Boat main thread 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "boat.h" 

#include "ws2812_config.h" 

//=======================================================================================


//=======================================================================================
// Dispatch 

// Event loop dispatch function for the main thread 
void Boat::BoatMainDispatch(Event event)
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
// States 

// Initialization state 
void Boat::MainInitState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.main_flags.standby_state = SET_BIT; 
        data.MainInitStateEntry(); 
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
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.init_state = CLEAR_BIT; 
        data.MainInitStateExit(); 
    }
}


// Standby state 
void Boat::MainStandbyState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.MainStandbyStateEntry(); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            data.radio.CommandCheck(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.standby_state = CLEAR_BIT; 
        data.MainStandbyStateExit(); 
    }
}


// Auto state 
void Boat::MainAutoState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.MainAutoStateEntry(); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            data.radio.CommandCheck(data); 
            break; 
        
        case MainEvents::NAV_HEADING_CALC: 
            data.navigation.HeadingCalc(data); 
            break; 

        case MainEvents::NAV_LOCATION_CALC: 
            data.navigation.LocationCalc(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.auto_state = CLEAR_BIT; 
        data.MainAutoStateExit(); 
    }
}


// Manual state 
void Boat::MainManualState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.MainManualStateEntry(); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            data.radio.CommandCheck(data); 
            break; 
        
        case MainEvents::RADIO_CONNECTION: 
            data.rc.RadioConnectionCheck(data.radio.ConnectionStatus()); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.manual_state = CLEAR_BIT; 
        data.MainManualStateExit(); 
    }
}


// Low Power state 
void Boat::MainLowPwrState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.MainLowPwrStateEntry(); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            data.radio.CommandCheck(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.low_pwr_state = CLEAR_BIT; 
        data.MainLowPwrStateExit(); 
    }
}


// Fault state 
void Boat::MainFaultState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.MainFaultStateEntry(); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::RADIO_CHECK: 
            data.radio.CommandCheck(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    // State exit 
    if (data.main_flags.state_exit)
    {
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.fault_state = CLEAR_BIT; 
        data.MainFaultStateExit(); 
    }
}


// Reset state 
void Boat::MainResetState(Boat& data, Event event)
{
    // State entry 
    if (data.main_flags.state_entry)
    {
        data.main_flags.state_entry = CLEAR_BIT; 
        data.MainResetStateEntry(); 
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
        data.main_flags.state_exit = CLEAR_BIT; 
        data.main_flags.state_entry = SET_BIT; 
        data.main_flags.reset_state = CLEAR_BIT; 
        data.MainResetStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// State entry/exit 

// Init state entry 
void Boat::MainInitStateEntry(void)
{
    // Start software timers 
    xTimerStart(periodic_timer_100ms, 0); 
    xTimerStart(periodic_timer_1s, 0); 

    // Load a waypoint mission if it exists 
    navigation.LoadMission(); 
}


// Init state exit 
void Boat::MainInitStateExit(void)
{
    // 
}


// Standby state entry 
void Boat::MainStandbyStateEntry(void)
{
    navigation.ThrustersOff(); 
    LEDStrobeUpdate(ws2812_led_standby_not_ready); 
    radio.MainStandbyStateCmdEnable(SET_BIT); 
}


// Standby state exit 
void Boat::MainStandbyStateExit(void)
{
    LEDStrobeOff(); 
    radio.MainStandbyStateCmdEnable(CLEAR_BIT); 
}


// Auto state entry 
void Boat::MainAutoStateEntry(void)
{
    navigation.CurrentUpdate(boat); 
    LEDStrobeUpdate(ws2812_led_auto_strobe); 
    LEDUpdate(ws2812_led_auto_star, ws2812_led_auto_port); 
    radio.MainAutoStateCmdEnable(SET_BIT); 
}


// Auto state exit 
void Boat::MainAutoStateExit(void)
{
    navigation.ThrustersOff(); 
    LEDStrobeOff(); 
    LEDUpdate(ws2812_led_off, ws2812_led_off); 
    radio.MainAutoStateCmdEnable(CLEAR_BIT); 
}


// Manual state entry 
void Boat::MainManualStateEntry(void)
{
    LEDStrobeUpdate(ws2812_led_manual_strobe); 
    radio.MainManualStateCmdEnable(SET_BIT); 
}


// Manual state exit 
void Boat::MainManualStateExit(void)
{
    rc.ThrustersOff(); 
    LEDStrobeOff(); 
    radio.MainManualStateCmdEnable(CLEAR_BIT); 
}


// Low power state entry 
void Boat::MainLowPwrStateEntry(void)
{
    // Stop the software timers 
    xTimerStop(periodic_timer_100ms, 0); 
    xTimerStop(periodic_timer_1s, 0); 

    // If all the software timers are stopped then there will be no radio checks or 
    // LED updates. 
    
    LEDStrobeUpdate(ws2812_led_low_pwr); 
    radio.MainLowPwrStateCmdEnable(SET_BIT); 
}


// Low power state exit 
void Boat::MainLowPwrStateExit(void)
{
    LEDStrobeOff(); 
    radio.MainLowPwrStateCmdEnable(CLEAR_BIT); 
}


// Fault state entry 
void Boat::MainFaultStateEntry(void)
{
    radio.MainFaultStateCmdEnable(SET_BIT); 
}


// Fault state exit 
void Boat::MainFaultStateExit(void)
{
    radio.MainFaultStateCmdEnable(CLEAR_BIT); 
}


// Reset state entry 
void Boat::MainResetStateEntry(void)
{
    // Stop the software timers 
    xTimerStop(periodic_timer_100ms, 0); 
    xTimerStop(periodic_timer_1s, 0); 
}


// Reset state exit 
void Boat::MainResetStateExit(void)
{
    // 
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Queue an event for the main thread 
void Boat::MainEventQueue(Event event)
{
    main_event_info.event = event; 
    xQueueSend(main_event_info.ThreadEventQueue, (void *)&main_event_info.event, 0); 
}


// Main thread state change 
void Boat::MainStateChange(void)
{
    main_flags.state_exit = SET_BIT; 
    MainEventQueue((Event)MainEvents::NO_EVENT); 
}

//=======================================================================================
