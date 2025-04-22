/**
 * @file timers_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief "timers.c" mock functions 
 * 
 * @version 0.1
 * @date 2024-04-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "FreeRTOS.h" 
#include "timers.h" 
#include "portmacro.h" 
#include "projdefs.h" 

//=======================================================================================


//=======================================================================================
// Mock implementations 

BaseType_t xTimerGenericCommand(
    TimerHandle_t xTimer, 
    const BaseType_t xCommandID, 
    const TickType_t xOptionalValue, 
    BaseType_t * const pxHigherPriorityTaskWoken, 
    const TickType_t xTicksToWait )
{
	return pdTRUE;
}

//=======================================================================================
