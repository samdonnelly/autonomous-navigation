/**
 * @file vehicle.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Generic vehicle 
 * 
 * @version 0.1
 * @date 2025-02-26
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "vehicle.h" 

//=======================================================================================


//=======================================================================================
// Project interface 

// Constructor 
Vehicle::Vehicle(uint8_t vehicle_type) 
    : main_event(MainEvents::NO_EVENT), 
      comms_event(CommsEvents::NO_EVENT), 
      hardware(), 
      telemetry(vehicle_type), 
      navigation(), 
      control(), 
      memory(), 
      auxiliary()
{
    main_system_flags.state_entry = FLAG_SET; 
    main_system_flags.state_exit = FLAG_CLEAR; 
}


// Vehicle setup 
void Vehicle::Setup(void)
{
    // Initialize FreeRTOS scheduler 
    osKernelInitialize(); 

    // Set up the vehicle hardware (ex. UART, radio, etc); 
    hardware.HardwareSetup(); 

    // Thread definitions. Vehicle specific dispatch and callback function pointers get 
    // set in vehicle specific setup code. 
    main_event_info = (ThreadEventData)
    {
        (osThreadAttr_t)                                  // attr 
        {
            "Mainthread",                                 // name 
            CLEAR_SETTING,                                // attr_bits 
            nullptr,                                      // cb_mem 
            CLEAR_SETTING,                                // cb_size 
            nullptr,                                      // stack_mem 
            MAIN_STACK_SIZE,                              // stack_size 
            (osPriority_t)osPriorityLow,                  // priority 
            CLEAR_SETTING,                                // tz_module 
            CLEAR_SETTING                                 // reserved 
        }, 
        CLEAR_EVENT,                                      // event 
        xQueueCreate(MAIN_QUEUE_LEN, sizeof(uint32_t)),   // ThreadEventQueue 
        nullptr                                           // dispatch 
    }; 
    comms_event_info = (ThreadEventData)
    {
        (osThreadAttr_t)                                   // attr 
        {
            "CommsThread",                                 // name 
            CLEAR_SETTING,                                 // attr_bits 
            nullptr,                                       // cb_mem 
            CLEAR_SETTING,                                 // cb_size 
            nullptr,                                       // stack_mem 
            COMMS_STACK_SIZE,                              // stack_size 
            (osPriority_t)osPriorityNormal,                // priority 
            CLEAR_SETTING,                                 // tz_module 
            CLEAR_SETTING                                  // reserved 
        }, 
        CLEAR_EVENT,                                       // event 
        xQueueCreate(COMMS_QUEUE_LEN, sizeof(uint32_t)),   // ThreadEventQueue 
        nullptr                                            // dispatch 
    }; 

    periodic_timer_50ms = (TimerThreadData)
    {
        nullptr,                       // Handler 
        nullptr,                       // Callback 
        osTimerPeriodic,               // Timer type 
        PERIODIC_TIMER_50MS_PERIOD,    // Ticks 
        (osTimerAttr_t)                // Attributes 
        {
            "50ms",                    // Name 
            CLEAR_SETTING,             // Attribute bits - reserved 
            NULL,                      // Memory for control block 
            CLEAR_SETTING              // Control block memory size 
        }
    };
    periodic_timer_100ms = (TimerThreadData)
    {
        nullptr,                       // Handler 
        nullptr,                       // Callback 
        osTimerPeriodic,               // Timer type 
        PERIODIC_TIMER_100MS_PERIOD,   // Ticks 
        (osTimerAttr_t)                // Attributes 
        {
            "100ms",                   // Name 
            CLEAR_SETTING,             // Attribute bits - reserved 
            NULL,                      // Memory for control block 
            CLEAR_SETTING              // Control block memory size 
        }
    };
    periodic_timer_250ms = (TimerThreadData)
    {
        nullptr,                       // Handler 
        nullptr,                       // Callback 
        osTimerPeriodic,               // Timer type 
        PERIODIC_TIMER_250MS_PERIOD,   // Ticks 
        (osTimerAttr_t)                // Attributes 
        {
            "250ms",                   // Name 
            CLEAR_SETTING,             // Attribute bits - reserved 
            NULL,                      // Memory for control block 
            CLEAR_SETTING              // Control block memory size 
        }
    };
    periodic_timer_1s = (TimerThreadData)
    {
        nullptr,                       // Handler 
        nullptr,                       // Callback 
        osTimerPeriodic,               // Timer type 
        PERIODIC_TIMER_1S_PERIOD,      // Ticks 
        (osTimerAttr_t)                // Attributes 
        {
            "1s",                      // Name 
            CLEAR_SETTING,             // Attribute bits - reserved 
            NULL,                      // Memory for control block 
            CLEAR_SETTING              // Control block memory size 
        }
    }; 
    
    // Run vehicle specific setup code. This is done after thread definitions and before 
    // thread creation so vehicle specific dispatch and callback functions can be 
    // assigned and not overwritten. 
    VehicleSetup(); 

    // Create the threads 
    osThreadNew(eventLoop, (void *)&main_event_info, &main_event_info.attr); 
    osThreadNew(eventLoop, (void *)&comms_event_info, &comms_event_info.attr); 

    periodic_timer_50ms.handler = osTimerNew(
        periodic_timer_50ms.callback, 
        periodic_timer_50ms.type, 
        nullptr, 
        &periodic_timer_50ms.attributes); 
    
    periodic_timer_100ms.handler = osTimerNew(
        periodic_timer_100ms.callback, 
        periodic_timer_100ms.type, 
        nullptr, 
        &periodic_timer_100ms.attributes); 

    periodic_timer_250ms.handler = osTimerNew(
        periodic_timer_250ms.callback, 
        periodic_timer_250ms.type, 
        nullptr, 
        &periodic_timer_250ms.attributes); 

    periodic_timer_1s.handler = osTimerNew(
        periodic_timer_1s.callback, 
        periodic_timer_1s.type, 
        nullptr, 
        &periodic_timer_1s.attributes); 

    // Create mutex and semaphores 
    comms_mutex = xSemaphoreCreateMutex(); 
    telemetry_out_semaphore = osSemaphoreNew(TELEMETRY_OUT_SEMAPHORE_COUNT,   // Total 
                                             TELEMETRY_OUT_SEMAPHORE_COUNT,   // Initial 
                                             NULL);                           // Attributes 

    // Queue the first event to start the system 
    MainEventQueue((Event)MainEvents::INIT); 
}


// Vehicle loop 
void Vehicle::Loop(void)
{
    // Start the RTOS scheduler 
    osKernelStart(); 
}

//=======================================================================================


//=======================================================================================
// Helper functions 

// Queue an event for the main thread 
void Vehicle::MainEventQueue(Event event)
{
    main_event_info.event = event; 
    xQueueSend(main_event_info.ThreadEventQueue, (void *)&main_event_info.event, 0); 
}


// Main thread common events 
void Vehicle::MainCommonEvents(Vehicle::MainEvents &event)
{
    switch (event)
    {
        case MainEvents::IMU_UPDATE: 
            navigation.OrientationUpdate(*this); 
            break; 
        
        case MainEvents::GPS_UPDATE: 
            navigation.LocationUpdate(*this); 
            break; 
        
        case MainEvents::RC_UPDATE: 
            control.RCUpdate(*this); 
            break; 
        
        case MainEvents::TELEMETRY_DECODE: 
            telemetry.MessageDecode(*this); 
            break; 

        case MainEvents::TELEMETRY_ENCODE: 
            telemetry.MessageEncode(*this); 
            break; 
        
        default:
            break; 
    }
}


// Main thread state change 
void Vehicle::MainStateChange(void)
{
    main_system_flags.state_exit = FLAG_SET; 
    MainEventQueue((Event)MainEvents::NO_EVENT); 
}


// Main thread state enter 
void Vehicle::MainStateEnter(
    uint8_t state, 
    uint32_t &flags)
{
    // All of the state/mode flags are cleared so a state flag doesn't get set for a 
    // state that can't be moved to from the current state, but can for another state. 

    main_system_flags.state_entry = FLAG_CLEAR; 
    flags = RESET; 
    telemetry.MAVLinkHeartbeatSetMode(state); 
}


// Main thread state exit 
void Vehicle::MainStateExit(void)
{
    main_system_flags.state_exit = FLAG_CLEAR; 
    main_system_flags.state_entry = FLAG_SET; 
}


// Queue an event for the comms thread 
void Vehicle::CommsEventQueue(Event event)
{
    comms_event_info.event = event; 
    xQueueSend(comms_event_info.ThreadEventQueue, (void *)&comms_event_info.event, 0); 
}

//=======================================================================================
