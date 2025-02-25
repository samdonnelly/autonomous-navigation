/**
 * @file rtos.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief RTOS features 
 * 
 * @version 0.1
 * @date 2025-02-25
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "rtos.h" 
#include "portmacro.h" 

//=======================================================================================


//=======================================================================================
// Functions 

// Common event loop shared by all threads 
void eventLoop(void *thread_info)
{
    ThreadEventData *thread = (ThreadEventData *)thread_info; 

    // Event loop 
    while (1)
    {
        thread->event = CLEAR_EVENT; 
        xQueueReceive(thread->ThreadEventQueue, (void *)&thread->event, portMAX_DELAY); 
        thread->dispatch(thread->event); 
    }

    osThreadExit(); 
}

//=======================================================================================
