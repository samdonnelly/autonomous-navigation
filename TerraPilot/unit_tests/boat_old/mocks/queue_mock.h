/**
 * @file queue_mock.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief "queue.c" mock functions interface 
 * 
 * @version 0.1
 * @date 2024-04-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _QUEUE_MOCK_H_ 
#define _QUEUE_MOCK_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Include 

#include "FreeRTOS.h" 
#include "queue.h" 
#include "portmacro.h" 
#include "projdefs.h" 

//=======================================================================================


//=======================================================================================
// Mock functions 

/**
 * @brief Mock driver initialization 
 */
void QueueMockInit(void); 


/**
 * @brief Get next event from the mock queue 
 * 
 * @return uint8_t : event value 
 */
uint8_t QueueMockGetNextEvent(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _QUEUE_MOCK_H_ 
