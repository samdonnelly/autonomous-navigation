/**
 * @file tasks_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief "tasks.c" mock functions 
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
#include "task.h" 
#include "portmacro.h" 

//=======================================================================================


//=======================================================================================
// Mock implementations 

TickType_t xTaskGetTickCount( void )
{
	return 0;
}

//=======================================================================================