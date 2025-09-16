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

/**
 * @brief Event loop dispatch function for the main thread 
 * 
 * @details This function will be called whenever there's a main event queued up. Main 
 *          events get queued by periodic timers. In the main thread, events are executed 
 *          based on the state the vehicle is in. 
 * 
 * @param event : event number 
 */
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
        
        // Start software timers 
        osTimerStart(data.periodic_timer_50ms.handler, data.periodic_timer_50ms.ticks); 
        osTimerStart(data.periodic_timer_100ms.handler, data.periodic_timer_100ms.ticks); 
        osTimerStart(data.periodic_timer_250ms.handler, data.periodic_timer_250ms.ticks); 
        osTimerStart(data.periodic_timer_1s.handler, data.periodic_timer_1s.ticks); 
        
        // Load parameters and a waypoint mission if they exists in memory. 
        data.memory.ParameterLoad(data); 
        data.memory.MissionLoad(); 
    }
    
    data.main_event = (MainEvents)event;

    switch (data.main_event)
    {
        case MainEvents::INIT: 
            data.main_state_flags.hold_state = FLAG_SET; 
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

        data.control.ForceStop(data); 
    }

    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Manual state 

void Boat::MainManualState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::MANUAL_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

    switch (data.main_event)
    {
        case MainEvents::REMOTE_CONTROL: 
            data.control.RemoteControl(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT;
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 

        data.control.ForceStop(data); 
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
    data.MainCommonEvents(data.main_event);

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
// Steering state 

void Boat::MainSteeringState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::STEERING_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Loiter state 

void Boat::MainLoiterState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::LOITER_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Follow state 

void Boat::MainFollowState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::FOLLOW_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Simple state 

void Boat::MainSimpleState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::SIMPLE_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Dock state 

void Boat::MainDockState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::DOCK_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Circle state 

void Boat::MainCircleState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::CIRCLE_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

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
// Auto state 

void Boat::MainAutoState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::AUTO_STATE, data.main_state_flags.flags); 
        
        // The MAVLink MISSION_CURRENT message should be broadcast when in auto mode so 
        // the GCS knows what the vehicle is targeting. 
        data.telemetry.MAVLinkMissionCurrentEnable(); 
    }
    
    data.main_event = (MainEvents)event;
    data.MainCommonEvents(data.main_event);

    switch (data.main_event)
    {
        case MainEvents::COURSE_CORRECT: 
            data.navigation.CourseCorrection(data); 
            break; 

        case MainEvents::TARGET_ASSESS: 
            data.navigation.TargetAssess(data); 
            break; 
        
        default: 
            data.main_event = MainEvents::NO_EVENT;
            break; 
    }
    
    if (data.main_system_flags.state_exit)
    {
        data.MainStateExit(); 

        data.telemetry.MAVLinkMissionCurrentDisable(); 
        data.control.ForceStop(data); 
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
    data.MainCommonEvents(data.main_event);

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
// Smart RTL state 

void Boat::MainSmartRTLState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::SMART_RTL_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 
    data.MainCommonEvents(data.main_event);

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
// Guided state 

void Boat::MainGuidedState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.MainStateEnter((uint8_t)MainStates::GUIDED_STATE, data.main_state_flags.flags); 
    }
    
    data.main_event = (MainEvents)event; 
    data.MainCommonEvents(data.main_event);

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
// Fault state 

void Boat::MainFaultState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.main_system_flags.state_entry = FLAG_CLEAR; 
        data.main_state_flags.flags = RESET; 

        data.control.ForceStop(data); 
    }

    data.main_event = (MainEvents)event; 
    data.MainCommonEvents(data.main_event);

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
// Reset state 

void Boat::MainResetState(Boat& data, Event event)
{
    if (data.main_system_flags.state_entry)
    {
        data.main_system_flags.state_entry = FLAG_CLEAR; 
        data.main_state_flags.flags = RESET; 

        data.control.ForceStop(data); 

        // Stop the software timers 
        osTimerStop(data.periodic_timer_50ms.handler); 
        osTimerStop(data.periodic_timer_100ms.handler); 
        osTimerStop(data.periodic_timer_250ms.handler); 
        osTimerStop(data.periodic_timer_1s.handler); 
    }

    data.main_event = (MainEvents)event; 
    data.MainCommonEvents(data.main_event);

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

/**
 * @brief Main thread state selection 
 * 
 * @details Select a new vehicle state based on a state number. This function is used by 
 *          the telemetry and control modules to update the vehicle state. If the state 
 *          number matches the current state or is invalid then no action will be taken. 
 *          Even if a state number is new and valid, it will only result in a state 
 *          change if the state allows it (see state table). States listed below are 
 *          specific to the boat. 
 * 
 * @param state : state number 
 */
void Boat::MainStateSelect(uint8_t state)
{
    uint8_t state_select = FLAG_SET; 

    // If the state number is invalid or the requested state is the current state, then 
    // we can disregard the request. 
    if ((state == (uint8_t)main_state) || (state >= (uint8_t)MainStates::NUM_STATES))
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


/**
 * @brief Main thread state mapping to RC mode input 
 * 
 * @details This function is used by the control module to turn a mode input into a valid 
 *          state number for the boat. Modes are based on a PWM range read by the RC 
 *          receiver so they don't match state numbers on their own. After the state 
 *          number is mapped then MainStateSelect can be called to update the state. 
 * 
 * @see MainStateSelect 
 * 
 * @param mode : mode number 
 */
void Boat::MainStateRCModeMap(uint8_t &mode)
{
    switch (mode)
    {
        case VehicleControl::RCModes::RC_MODE1: 
            mode = (uint8_t)MainStates::MANUAL_STATE; 
            break; 

        case VehicleControl::RCModes::RC_MODE2: 
            mode = (uint8_t)MainStates::HOLD_STATE; 
            break; 

        case VehicleControl::RCModes::RC_MODE3: 
            mode = (uint8_t)MainStates::AUTO_STATE; 
            break; 
        
        default:   // Invalid mode number 
            mode = (uint8_t)MainStates::NUM_STATES; 
            break; 
    }
}

//=======================================================================================
