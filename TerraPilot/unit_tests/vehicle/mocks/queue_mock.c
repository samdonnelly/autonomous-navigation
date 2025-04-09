/**
 * @file queue_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Queue mock 
 * 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Includes 

#include "rtos.h" 

//=======================================================================================


//=======================================================================================
// Mock data 

typedef struct QueueDefinition 
{
    // 
} 
Queue_t; 

//=======================================================================================


//=======================================================================================
// Mock functions 

BaseType_t xQueueSemaphoreTake(
    QueueHandle_t xQueue, 
    TickType_t xTicksToWait)
{
    return pdTRUE; 
}


BaseType_t xQueueGenericSend(
    QueueHandle_t xQueue,
    const void * const pvItemToQueue,
    TickType_t xTicksToWait,
    const BaseType_t xCopyPosition)
{
    return pdTRUE; 
}


QueueHandle_t xQueueCreateMutex( const uint8_t ucQueueType )
{
    QueueHandle_t xNewQueue;
    const UBaseType_t uxMutexLength = ( UBaseType_t ) 1; 
    const UBaseType_t uxMutexSize = ( UBaseType_t ) 0;

    xNewQueue = xQueueGenericCreate( uxMutexLength, uxMutexSize, ucQueueType );

    return xNewQueue; 
}


QueueHandle_t xQueueGenericCreate(
    const UBaseType_t uxQueueLength,
    const UBaseType_t uxItemSize,
    const uint8_t ucQueueType)
{
    Queue_t *pxNewQueue = NULL; 

    return pxNewQueue; 
}


BaseType_t xQueueReceive(
    QueueHandle_t xQueue,
    void * const pvBuffer,
    TickType_t xTicksToWait)
{
    return pdTRUE; 
}

//=======================================================================================
