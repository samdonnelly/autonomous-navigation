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
        default: 
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
        case MainEvents::NAV_CALCS: 
            break; 
        
        default: 
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
        case MainEvents::REMOTE_CONTROL: 
            break; 
        
        default: 
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
        default: 
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
        default: 
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
    // Start the 100ms software timer 
    xTimerStart(periodic_timer_100ms, 0); 
}


// Init state exit 
void Boat::MainInitStateExit(void)
{
    // 
}


// Standby state entry 
void Boat::MainStandbyStateEntry(void)
{
    LEDStrobeUpdate(ws2812_led_standby_not_ready); 
    // If there is a position lock then light up an LED 
}


// Standby state exit 
void Boat::MainStandbyStateExit(void)
{
    LEDStrobeOff(); 
    // Turn off the position lock LED 
}


// Auto state entry 
void Boat::MainAutoStateEntry(void)
{
    LEDStrobeUpdate(ws2812_led_auto_strobe); 
    LEDUpdate(ws2812_led_auto_star, ws2812_led_auto_port); 
}


// Auto state exit 
void Boat::MainAutoStateExit(void)
{
    LEDStrobeOff(); 
    LEDUpdate(ws2812_led_off, ws2812_led_off); 
}


// Manual state entry 
void Boat::MainManualStateEntry(void)
{
    LEDStrobeUpdate(ws2812_led_manual_strobe); 
}


// Manual state exit 
void Boat::MainManualStateExit(void)
{
    LEDStrobeOff(); 
}


// Low power state entry 
void Boat::MainLowPwrStateEntry(void)
{
    LEDStrobeUpdate(ws2812_led_low_pwr); 
}


// Low power state exit 
void Boat::MainLowPwrStateExit(void)
{
    LEDStrobeOff(); 
}


// Fault state entry 
void Boat::MainFaultStateEntry(void)
{
    // Stop the 100ms software timer 
    xTimerStop(periodic_timer_100ms, 0); 
}


// Fault state exit 
void Boat::MainFaultStateExit(void)
{
    // 
}


// Reset state entry 
void Boat::MainResetStateEntry(void)
{
    // Stop the 100ms software timer 
    xTimerStop(periodic_timer_100ms, 0); 
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
