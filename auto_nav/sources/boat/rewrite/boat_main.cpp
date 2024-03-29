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

    boat.main_state_table[(uint8_t)state](&boat, event); 
    boat.main_state = state; 
}

//=======================================================================================


//=======================================================================================
// States 

// Initialization state 
void Boat::MainInitState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.fault_state || 
        data->main_flags.standby_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.init_state = CLEAR_BIT; 
    }
}


// Standby state 
void Boat::MainStandbyState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.fault_state || 
        data->main_flags.low_pwr_state || 
        data->main_flags.auto_state || 
        data->main_flags.manual_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.standby_state = CLEAR_BIT; 
    }
}


// Auto state 
void Boat::MainAutoState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.fault_state || 
        data->main_flags.low_pwr_state || 
        data->main_flags.standby_state || 
        data->main_flags.manual_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.auto_state = CLEAR_BIT; 
    }
}


// Manual state 
void Boat::MainManualState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.fault_state || 
        data->main_flags.low_pwr_state || 
        data->main_flags.standby_state || 
        data->main_flags.auto_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.manual_state = CLEAR_BIT; 
    }
}


// Low Power state 
void Boat::MainLowPwrState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.standby_state || 
        data->main_flags.reset_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.low_pwr_state = CLEAR_BIT; 
    }
}


// Fault state 
void Boat::MainFaultState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.reset_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.fault_state = CLEAR_BIT; 
    }
}


// Reset state 
void Boat::MainResetState(Boat *data, Event event)
{
    // State entry 
    if (data->main_flags.state_entry)
    {
        data->main_flags.state_entry = CLEAR_BIT; 
    }

    data->main_event = (MainEvents)event; 

    switch (data->main_event)
    {
        default: 
            break; 
    }

    // State exit 
    if (data->main_flags.init_state)
    {
        data->main_flags.state_entry = SET_BIT; 
        data->main_flags.reset_state = CLEAR_BIT; 
    }
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

//=======================================================================================
