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

// Vehicle setup 
void Vehicle::Setup(void)
{
    // Set up the vehicle hardware (ex. UART, radio, etc); 
    hardware.HardwareSetup(); 

    // Thread definitions. Vehicle specific dispatch and callback function pointers get 
    // set in vehicle specific setup code. 
    main_event_info = 
    {
        .attr = { .name = "MainThread", 
                  .attr_bits = CLEAR_SETTING, 
                  .cb_mem = nullptr, 
                  .cb_size = CLEAR_SETTING, 
                  .stack_mem = nullptr, 
                  .stack_size = MAIN_STACK_SIZE, 
                  .priority = (osPriority_t)osPriorityLow, 
                  .tz_module = CLEAR_SETTING, 
                  .reserved = CLEAR_SETTING }, 
        .event = CLEAR_EVENT, 
        .ThreadEventQueue = xQueueCreate(MAIN_QUEUE_LEN, sizeof(uint32_t)), 
        .dispatch = nullptr 
    }; 
    comms_event_info = 
    {
        .attr = { .name = "CommsThread", 
                  .attr_bits = CLEAR_SETTING, 
                  .cb_mem = nullptr, 
                  .cb_size = CLEAR_SETTING, 
                  .stack_mem = nullptr, 
                  .stack_size = COMMS_STACK_SIZE, 
                  .priority = (osPriority_t)osPriorityNormal, 
                  .tz_module = CLEAR_SETTING, 
                  .reserved = CLEAR_SETTING }, 
        .event = CLEAR_EVENT, 
        .ThreadEventQueue = xQueueCreate(COMMS_QUEUE_LEN, sizeof(uint32_t)), 
        .dispatch = nullptr 
    }; 

    periodic_timer_100ms = 
    {
        .handler = nullptr, 
        .name = "100ms", 
        .ticks = PERIODIC_TIMER_100MS_PERIOD, 
        .reload = pdTRUE, 
        .id = 0, 
        .callback = nullptr 
    }; 
    periodic_timer_250ms = 
    {
        .handler = nullptr, 
        .name = "250ms", 
        .ticks = PERIODIC_TIMER_250MS_PERIOD, 
        .reload = pdTRUE, 
        .id = 1, 
        .callback = nullptr 
    }; 
    periodic_timer_1s = 
    {
        .handler = nullptr, 
        .name = "1s", 
        .ticks = PERIODIC_TIMER_1S_PERIOD, 
        .reload = pdTRUE, 
        .id = 2, 
        .callback = nullptr 
    }; 
    
    // Run vehicle specific setup code. Thread creation is done after this because the 
    // vehicle specific code assigns vehicle specific dispatch and callback functions. 
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
        (void *)periodic_timer_100ms.id, 
        periodic_timer_100ms.callback); 

    periodic_timer_250ms.handler = xTimerCreate(
        periodic_timer_250ms.name, 
        periodic_timer_250ms.ticks, 
        periodic_timer_250ms.reload, 
        (void *)periodic_timer_250ms.id, 
        periodic_timer_250ms.callback); 
    
    periodic_timer_1s.handler = xTimerCreate(
        periodic_timer_1s.name, 
        periodic_timer_1s.ticks, 
        periodic_timer_1s.reload, 
        (void *)periodic_timer_1s.id, 
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


// Main thread state change 
void Vehicle::MainStateChange(void)
{
    main_system_flags.state_exit = FLAG_SET; 
    MainEventQueue((Event)MainEvents::NO_EVENT); 
}


// Queue an event for the comms thread 
void Vehicle::CommsEventQueue(Event event)
{
    comms_event_info.event = event; 
    xQueueSend(comms_event_info.ThreadEventQueue, (void *)&comms_event_info.event, 0); 
}

//=======================================================================================


//=======================================================================================
// Wrapper functions 

// Queue a TELEMETRY_WRITE event for the main thread 
void Vehicle::CommsEventQueueTelemetryWrite(void)
{
    CommsEventQueue((Event)CommsEvents::TELEMETRY_WRITE); 
}

//=======================================================================================
