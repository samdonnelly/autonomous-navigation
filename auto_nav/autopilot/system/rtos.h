/**
 * @file rtos.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief RTOS features interface 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef _RTOS_H_ 
#define _RTOS_H_ 

//=======================================================================================
// Includes 

#include "FreeRTOSConfig.h" 
#include "FreeRTOS.h" 
#include "cmsis_os2.h" 
#include "queue.h" 
#include "semphr.h" 
#include "timers.h" 
#include "portmacro.h" 

//=======================================================================================


//=======================================================================================
// Thread configuration 

// Main thread memory 
#define MAIN_STACK_MULTIPLIER 8 
#define MAIN_STACK_SIZE configMINIMAL_STACK_SIZE * MAIN_STACK_MULTIPLIER 
#define MAIN_QUEUE_LEN 10 

// Communication thread memory 
#define COMMS_STACK_MULTIPLIER 8 
#define COMMS_STACK_SIZE configMINIMAL_STACK_SIZE * COMMS_STACK_MULTIPLIER 
#define COMMS_QUEUE_LEN 10 

// Software timers thread (5ms tick period) 
#define PERIODIC_TIMER_100MS_PERIOD 20   // Ticks 
#define PERIODIC_TIMER_250MS_PERIOD 50   // Ticks 
#define PERIODIC_TIMER_1S_PERIOD 200     // Ticks 

// Thread synchronization 
#define TELEMETRY_OUT_SEMAPHORE_COUNT 1 

//=======================================================================================


//=======================================================================================
// Datatypes 

typedef uint8_t Event; 

//=======================================================================================


//=======================================================================================
// Data 

// Thread event info 
struct ThreadEventData 
{
    osThreadAttr_t attr;              // Thread attributes 
    uint8_t event;                    // Event index 
    QueueHandle_t ThreadEventQueue;   // Queue 
    void (*dispatch)(Event event);    // Dispatch function 
}; 


// Timer thread info 
struct TimerThreadData 
{
    void *handler;              // Handler / ID 
    osTimerFunc_t callback;     // Callback function pointer 
    osTimerType_t type;         // Timer type 
    uint32_t ticks;             // Period of timer (ticks) 
    osTimerAttr_t attributes;   // Attributes 
};

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Common event loop task function for all threads (FreeRTOS format) 
 * 
 * @param thread_info : void pointer to a 'ThreadEventData' object (see above) 
 */
void eventLoop(void *thread_info); 

//=======================================================================================

#endif   // _RTOS_H_ 
