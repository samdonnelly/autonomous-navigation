/**
 * @file timers_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief 
 * 
 * @version 0.1
 * @date 2025-03-28
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//=======================================================================================
// Include 

#include "rtos.h" 

//=======================================================================================


//=======================================================================================
// Mock data 

typedef struct tmrTimerControl
{
    // 
}
Timer_t; 

//=======================================================================================


//=======================================================================================
// Mock functions 

TimerHandle_t xTimerCreate( const char * const pcTimerName,
    const TickType_t xTimerPeriodInTicks,
    const BaseType_t xAutoReload,
    void * const pvTimerID,
    TimerCallbackFunction_t pxCallbackFunction )
{
    Timer_t *pxNewTimer = NULL; 
    return pxNewTimer; 
}

//=======================================================================================
