/**
 * @file queue_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief "queue.c" mock function 
 * 
 * @version 0.1
 * @date 2024-03-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Include 

#include "FreeRTOS.h" 
#include "queue.h" 
#include "portmacro.h" 
#include "projdefs.h" 

//=======================================================================================


//=======================================================================================
// Mock implementations 

BaseType_t xQueueGenericSend(
    QueueHandle_t xQueue, 
    const void * const pvItemToQueue, 
    TickType_t xTicksToWait, 
    const BaseType_t xCopyPosition)
{
    return pdTRUE; 
}

//=======================================================================================