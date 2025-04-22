/**
 * @file cmsis_os2_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief CMSIS OS v2 mock 
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
// Mock functions 

osStatus_t osKernelInitialize(void)
{
    return osOK; 
}


osThreadId_t osThreadNew(
    osThreadFunc_t func, 
    void *argument, 
    const osThreadAttr_t *attr)
{
    TaskHandle_t hTask = { 0 }; 
    return ((osThreadId_t)hTask); 
}


osStatus_t osKernelStart(void)
{
    return osOK; 
}


osSemaphoreId_t osSemaphoreNew(
    uint32_t max_count, 
    uint32_t initial_count, 
    const osSemaphoreAttr_t *attr)
{
    SemaphoreHandle_t hSemaphore = NULL; 
    return ((osSemaphoreId_t)hSemaphore); 
}


osStatus_t osSemaphoreAcquire(
    osSemaphoreId_t semaphore_id, 
    uint32_t timeout)
{
    return osOK; 
}


osStatus_t osSemaphoreRelease(osSemaphoreId_t semaphore_id) 
{
    return osOK; 
}


osTimerId_t osTimerNew(
    osTimerFunc_t func, 
    osTimerType_t type, 
    void *argument, 
    const osTimerAttr_t *attr)
{
    TimerHandle_t hTimer = NULL; 
    return ((osTimerId_t)hTimer); 
}


osStatus_t osTimerStart (osTimerId_t timer_id, uint32_t ticks) 
{
    return osOK; 
}


osStatus_t osTimerStop (osTimerId_t timer_id) 
{
    return osOK; 
}

//=======================================================================================