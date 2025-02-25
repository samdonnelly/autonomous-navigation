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

#include "system_tools.h" 
#include "FreeRTOS.h" 
#include "cmsis_os2.h" 
#include "queue.h" 
#include "semphr.h" 
#include "timers.h" 

//=======================================================================================


//=======================================================================================
// Datatypes 

typedef uint8_t Event; 

//=======================================================================================


//=======================================================================================
// Data 

// Thread Event Info 
typedef struct 
{
    osThreadAttr_t attr;              // Thread attributes 
    uint8_t event;                    // Event index 
    QueueHandle_t ThreadEventQueue;   // Queue 
    void (*dispatch)(Event event);    // Dispatch function 
} 
ThreadEventData; 

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
