/**
 * @file queue_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief "queue.c" mock functions 
 * 
 * @version 0.1
 * @date 2024-03-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Include 

#include "queue_mock.h" 
#include "tools.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define QUEUE_MOCK_LEN 10 

//=======================================================================================


//=======================================================================================
// Prototypes 

// Add an event to the mock queue 
void QueueMockSetEvent(uint8_t event); 

//=======================================================================================


//=======================================================================================
// Variables 

// Queue Mock data 
typedef struct 
{
    // Circular buffer to store event queue 
    uint8_t event_buff[QUEUE_MOCK_LEN]; 
    uint8_t event_buff_set_index; 
    uint8_t event_buff_get_index; 
    uint8_t event_buff_index_delta; 
}
queue_mock_data; 

// Queue Mock data object 
static queue_mock_data mock_data; 

//=======================================================================================


//=======================================================================================
// Driver functions 

BaseType_t xQueueGenericSend(
    QueueHandle_t xQueue, 
    const void * const pvItemToQueue, 
    TickType_t xTicksToWait, 
    const BaseType_t xCopyPosition)
{
    if (pvItemToQueue != NULL)
    {
        QueueMockSetEvent(*((uint8_t *)pvItemToQueue)); 
    }
    return pdTRUE; 
}


BaseType_t xQueueSemaphoreTake(
    QueueHandle_t xQueue, 
    TickType_t xTicksToWait)
{
    return pdPASS; 
}

//=======================================================================================


//=======================================================================================
// Mock functions 

// Mock driver initialization 
void QueueMockInit(void)
{
    memset((void *)mock_data.event_buff, CLEAR, sizeof(mock_data.event_buff)); 
    mock_data.event_buff_set_index = CLEAR; 
    mock_data.event_buff_get_index = CLEAR; 
    mock_data.event_buff_index_delta = CLEAR; 
}


// Add an event to the mock queue 
void QueueMockSetEvent(uint8_t event)
{
    if (mock_data.event_buff_set_index >= QUEUE_MOCK_LEN)
    {
        mock_data.event_buff_set_index = CLEAR; 
    }

    mock_data.event_buff_index_delta++; 
    mock_data.event_buff[mock_data.event_buff_set_index++] = event; 
}


// Get next event from the mock queue 
uint8_t QueueMockGetNextEvent(void)
{
    if (mock_data.event_buff_index_delta == 0)
    {
        return ~0; 
    }

    if (mock_data.event_buff_get_index >= QUEUE_MOCK_LEN)
    {
        mock_data.event_buff_get_index = CLEAR; 
    }

    mock_data.event_buff_index_delta--; 
    return mock_data.event_buff[mock_data.event_buff_get_index++]; 
}

//=======================================================================================