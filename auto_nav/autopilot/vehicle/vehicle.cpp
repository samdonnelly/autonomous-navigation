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
            "Mainthread",                  // name 
            CLEAR_SETTING,                 // attr_bits 
            nullptr,                       // cb_mem 
            CLEAR_SETTING,                 // cb_size 
            nullptr,                       // stack_mem 
            MAIN_STACK_SIZE,               // stack_size 
            (osPriority_t)osPriorityLow,   // priority 
            CLEAR_SETTING,                 // tz_module 
            CLEAR_SETTING                  // reserved 
        }, 
        CLEAR_EVENT,                                      // event 
        xQueueCreate(MAIN_QUEUE_LEN, sizeof(uint32_t)),   // ThreadEventQueue 
        nullptr                                           // dispatch 
    }; 
    comms_event_info = (ThreadEventData)
    {
        (osThreadAttr_t)                                   // attr 
        {
            "CommsThread",                    // name 
            CLEAR_SETTING,                    // attr_bits 
            nullptr,                          // cb_mem 
            CLEAR_SETTING,                    // cb_size 
            nullptr,                          // stack_mem 
            COMMS_STACK_SIZE,                 // stack_size 
            (osPriority_t)osPriorityNormal,   // priority 
            CLEAR_SETTING,                    // tz_module 
            CLEAR_SETTING                     // reserved 
        }, 
        CLEAR_EVENT,                                       // event 
        xQueueCreate(COMMS_QUEUE_LEN, sizeof(uint32_t)),   // ThreadEventQueue 
        nullptr                                            // dispatch 
    }; 

    periodic_timer_100ms = (TimerThreadData)
    {
        nullptr,                       // handler
        "100ms",                       // name 
        PERIODIC_TIMER_100MS_PERIOD,   // ticks 
        pdTRUE,                        // reload 
        0,                             // id 
        nullptr                        // callback 
    }; 
    periodic_timer_250ms = (TimerThreadData)
    {
        nullptr,                       // handler
        "250ms",                       // name 
        PERIODIC_TIMER_250MS_PERIOD,   // ticks 
        pdTRUE,                        // reload 
        1,                             // id 
        nullptr                        // callback 
    }; 
    periodic_timer_1s = (TimerThreadData)
    {
        nullptr,                       // handler
        "1s",                          // name 
        PERIODIC_TIMER_1S_PERIOD,      // ticks 
        pdTRUE,                        // reload 
        2,                             // id 
        nullptr                        // callback 
    }; 
    
    // Run vehicle specific setup code. This is done after thread definitions and before 
    // thread creation so vehicle specific dispatch and callback functions can be 
    // assigned and not overwritten. 
    VehicleSetup(); 

    // Create the threads 
    osThreadNew(eventLoop, (void *)&main_event_info, &main_event_info.attr); 
    osThreadNew(eventLoop, (void *)&comms_event_info, &comms_event_info.attr); 
    // Check that the thread creation worked 

    // Create software timers (executed within the software timers thread) 
    periodic_timer_100ms.handler = xTimerCreate(
        periodic_timer_100ms.name, 
        periodic_timer_100ms.ticks, 
        periodic_timer_100ms.reload, 
        (void *)&periodic_timer_100ms.id, 
        periodic_timer_100ms.callback); 

    periodic_timer_250ms.handler = xTimerCreate(
        periodic_timer_250ms.name, 
        periodic_timer_250ms.ticks, 
        periodic_timer_250ms.reload, 
        (void *)&periodic_timer_250ms.id, 
        periodic_timer_250ms.callback); 
    
    periodic_timer_1s.handler = xTimerCreate(
        periodic_timer_1s.name, 
        periodic_timer_1s.ticks, 
        periodic_timer_1s.reload, 
        (void *)&periodic_timer_1s.id, 
        periodic_timer_1s.callback); 

    // Create mutex 
    comms_mutex = xSemaphoreCreateMutex(); 
    telemetry_out_mutex = xSemaphoreCreateMutex(); 

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
        case MainEvents::TELEMETRY_DECODE: 
            telemetry.MAVLinkMessageDecode(*this); 
            break; 

        case MainEvents::TELEMETRY_ENCODE: 
            telemetry.MAVLinkMessageEncode(*this); 
            break; 
        
        default: 
            event = MainEvents::NO_EVENT; 
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
