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
            if (boat.main_state_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_state_flags.hold_state)
            {
                state = MainStates::HOLD_STATE; 
            }
            break; 

        case MainStates::HOLD_STATE: 
            if (boat.main_state_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_state_flags.auto_state)
            {
                state = MainStates::AUTO_STATE; 
            }
            else if (boat.main_state_flags.manual_state)
            {
                state = MainStates::MANUAL_STATE; 
            }
            break; 
            
        case MainStates::MANUAL_STATE: 
            if (boat.main_state_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_state_flags.hold_state)
            {
                state = MainStates::HOLD_STATE; 
            }
            else if (boat.main_state_flags.auto_state)
            {
                state = MainStates::AUTO_STATE; 
            }
            break; 

        case MainStates::ACRO_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::STEERING_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::LOITER_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::FOLLOW_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::SIMPLE_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::DOCK_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::CIRCLE_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::AUTO_STATE: 
            if (boat.main_state_flags.fault_state)
            {
                state = MainStates::FAULT_STATE; 
            }
            else if (boat.main_state_flags.hold_state)
            {
                state = MainStates::HOLD_STATE; 
            }
            else if (boat.main_state_flags.manual_state)
            {
                state = MainStates::MANUAL_STATE; 
            }
            break; 

        case MainStates::RTL_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::SMART_RTL_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::GUIDED_STATE: 
            state = MainStates::INIT_STATE; 
            break; 

        case MainStates::FAULT_STATE: 
            if (boat.main_state_flags.reset_state)
            {
                state = MainStates::RESET_STATE; 
            }
            break; 

        case MainStates::RESET_STATE: 
            if (boat.main_state_flags.init_state)
            {
                state = MainStates::INIT_STATE; 
            }
            break; 
        
        default: 
            state = MainStates::INIT_STATE; 
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
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::INIT_STATE, data.main_state_flags.flags); 
        
        data.main_state_flags.hold_state = FLAG_SET; 

        // Start software timers 
        osTimerStart(data.periodic_timer_100ms.handler, data.periodic_timer_100ms.ticks); 
        osTimerStart(data.periodic_timer_250ms.handler, data.periodic_timer_250ms.ticks); 
        osTimerStart(data.periodic_timer_1s.handler, data.periodic_timer_1s.ticks); 

        // Load a waypoint mission if it exists 
        data.memory.MissionLoad(); 
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
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Hold state 

void Boat::MainHoldState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::HOLD_STATE, data.main_state_flags.flags); 

        // navigation.ThrustersOff(); 
        // LEDStrobeUpdate(ws2812_led_standby_not_ready); 
        // radio.MainStandbyStateCmdEnable(FLAG_SET); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 

        // LEDStrobeOff(); 
        // radio.MainStandbyStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// Manual state 

void Boat::MainManualState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::MANUAL_STATE, data.main_state_flags.flags); 

        // LEDStrobeUpdate(ws2812_led_manual_strobe); 
        // radio.MainManualStateCmdEnable(FLAG_SET); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        // case MainEvents::RADIO_CONNECTION: 
            // data.rc.RadioConnectionCheck(data.radio.ConnectionStatus()); 
            // break; 

        // case MainEvents::REMOTE_CONTROL: 
        //     // Check for RC receiver input to control the vehicle and simultaneously 
        //     // check that there's still an RC radio connection. 
        //     break; 
        
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 

        // rc.ThrustersOff(); 
        // LEDStrobeOff(); 
        // radio.MainManualStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// Acro state 

void Boat::MainAcroState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::ACRO_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Steering state 

void Boat::MainSteeringState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::STEERING_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Loiter state 

void Boat::MainLoiterState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::LOITER_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Follow state 

void Boat::MainFollowState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::FOLLOW_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Simple state 

void Boat::MainSimpleState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::SIMPLE_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Dock state 

void Boat::MainDockState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::DOCK_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Circle state 

void Boat::MainCircleState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::CIRCLE_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Auto state 

void Boat::MainAutoState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::AUTO_STATE, data.main_state_flags.flags); 
        
        // The MAVLink MISSION_CURRENT message should be broadcast when in auto mode so 
        // the GCS knows what the vehicle is targeting. 
        data.telemetry.MAVLinkMissionCurrentEnable(); 

        // navigation.CurrentUpdate(boat); 
        // LEDStrobeUpdate(ws2812_led_auto_strobe); 
        // LEDUpdate(ws2812_led_auto_star, ws2812_led_auto_port); 
        // radio.MainAutoStateCmdEnable(FLAG_SET); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        case MainEvents::NAV_HEADING_CALC: 
            // data.navigation.HeadingCalc(data); 
            break; 
        
        case MainEvents::NAV_LOCATION_CALC: 
            // data.navigation.LocationCalc(data); 
            break; 
        
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 

        data.telemetry.MAVLinkMissionCurrentDisable(); 

        // navigation.ThrustersOff(); 
        // LEDStrobeOff(); 
        // LEDUpdate(ws2812_led_off, ws2812_led_off); 
        // radio.MainAutoStateCmdEnable(FLAG_CLEAR); 
    }
}

//=======================================================================================


//=======================================================================================
// RTL state 

void Boat::MainRTLState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::RTL_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Smart RTL state 

void Boat::MainSmartRTLState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::SMART_RTL_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Guided state 

void Boat::MainGuidedState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::GUIDED_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Fault state 

void Boat::MainFaultState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.main_system_flags.state_entry = FLAG_CLEAR; 
        data.main_state_flags.flags = RESET; 

        // radio.MainFaultStateCmdEnable(FLAG_SET); 

        //==================================================
        // From low power state 

        // Stop the software timers 
        // xTimerStop(data.periodic_timer_100ms.handler, 0); 
        // xTimerStop(data.periodic_timer_1s.handler, 0); 

        // If all the software timers are stopped then there will be no radio checks or 
        // LED updates. 
    
        // LEDStrobeUpdate(ws2812_led_low_pwr); 
        // radio.MainLowPwrStateCmdEnable(FLAG_SET); 

        //==================================================
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.MainCommonEvents(data.main_event); 
            break; 
    }

    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 

        // radio.MainFaultStateCmdEnable(FLAG_CLEAR); 

        //==================================================
        // From low power state 

        // LEDStrobeOff(); 
        // radio.MainLowPwrStateCmdEnable(FLAG_CLEAR); 

        //==================================================
    }
}

//=======================================================================================


//=======================================================================================
// Reset state 

void Boat::MainResetState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.main_system_flags.state_entry = FLAG_CLEAR; 
        data.main_state_flags.flags = RESET; 

        // Stop the software timers 
        osTimerStop(data.periodic_timer_100ms.handler); 
        osTimerStop(data.periodic_timer_250ms.handler); 
        osTimerStop(data.periodic_timer_1s.handler); 
    }

    data.main_event = (MainEvents)event; 

    switch (data.main_event)
    {
        default: 
            data.main_event = MainEvents::NO_EVENT; 
            break; 
    }

    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 
    }
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Main thread state selection 
void Boat::MainStateSelect(uint8_t state)
{
    uint8_t state_select = FLAG_SET; 

    if (state >= (uint8_t)MainStates::NUM_STATES)
    {
        return; 
    }

    switch ((MainStates)state)
    {
        case MainStates::MANUAL_STATE: 
            main_state_flags.manual_state = FLAG_SET; 
            break; 

        case MainStates::ACRO_STATE: 
            main_state_flags.acro_state = FLAG_SET; 
            break; 

        case MainStates::STEERING_STATE: 
            main_state_flags.steering_state = FLAG_SET; 
            break; 

        case MainStates::HOLD_STATE: 
            main_state_flags.hold_state = FLAG_SET; 
            break; 

        case MainStates::LOITER_STATE: 
            main_state_flags.loiter_state = FLAG_SET; 
            break; 

        case MainStates::FOLLOW_STATE: 
            main_state_flags.follow_state = FLAG_SET; 
            break; 

        case MainStates::SIMPLE_STATE: 
            main_state_flags.simple_state = FLAG_SET; 
            break; 

        case MainStates::DOCK_STATE: 
            main_state_flags.dock_state = FLAG_SET; 
            break; 

        case MainStates::CIRCLE_STATE: 
            main_state_flags.circle_state = FLAG_SET; 
            break; 

        case MainStates::AUTO_STATE: 
            main_state_flags.auto_state = FLAG_SET; 
            break; 

        case MainStates::RTL_STATE: 
            main_state_flags.rtl_state = FLAG_SET; 
            break; 

        case MainStates::SMART_RTL_STATE: 
            main_state_flags.smart_rtl_state = FLAG_SET; 
            break; 

        case MainStates::GUIDED_STATE: 
            main_state_flags.guided_state = FLAG_SET; 
            break; 

        case MainStates::INIT_STATE: 
            main_state_flags.init_state = FLAG_SET; 
            break; 
        
        default:   // Unused and non-ArduPilot states/modes 
            state_select = FLAG_CLEAR; 
            break; 
    }

    if (state_select)
    {
        MainStateChange(); 
    }
}

//=======================================================================================
