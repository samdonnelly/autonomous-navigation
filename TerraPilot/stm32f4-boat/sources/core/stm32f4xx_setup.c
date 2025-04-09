/**
 * @file stm32f4xx_setup.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief STM32F4xx setup implementation 
 * 
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "stm32f4xx_setup.h"
#include "hardware_config.h" 
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

//=======================================================================================


//=======================================================================================
// Global variables 

#if FREERTOS_ENABLE 
TIM_HandleTypeDef htim11; 
#endif   // FREERTOS_ENABLE 

//=======================================================================================


//=======================================================================================
// Timer functions 

#if FREERTOS_ENABLE 

// HAL time base based on the hardware TIM. 

/**
 * @brief Configure a non-SysTick timer as the HAL time base source 
 * 
 * @details The time source is configured  to have 1ms time base with a dedicated
 *          Tick interrupt priority. 
 *          
 *          This function is called  automatically at the beginning of program after
 *          reset by HAL_Init() or at any time when clock is configured, by HAL_RCC_ClockConfig().
 * 
 * @param TickPriority : Tick interrupt priority 
 * @return HAL_StatusTypeDef : HAL status 
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t uwTimclock = 0U, uwPrescalerValue = 0U, pFLatency;
    HAL_StatusTypeDef status;

    // Enable TIM11 clock 
    __HAL_RCC_TIM11_CLK_ENABLE();

    // Get clock configuration 
    HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

    // Compute TIM11 clock 
    uwTimclock = HAL_RCC_GetPCLK2Freq();

    // Compute the prescaler value to have TIM11 counter clock equal to 1MHz 
    uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

    // Initialize TIM11 
    htim11.Instance = TIM11;

    // Initialize TIMx peripheral as follow:
    // - Period = [(TIM11CLK/1000) - 1]. to have a (1/1000) s time base.
    // - Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
    // - ClockDivision = 0
    // - Counter direction = Up
    htim11.Init.Period = (1000000U / 1000U) - 1U;
    htim11.Init.Prescaler = uwPrescalerValue;
    htim11.Init.ClockDivision = 0;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    status = HAL_TIM_Base_Init(&htim11); 

    if (status == HAL_OK)
    {
        // Start the TIM time Base generation in interrupt mode 
        status = HAL_TIM_Base_Start_IT(&htim11); 
        
        if (status == HAL_OK)
        {
            // Enable the TIM11 global Interrupt 
            HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
            // Configure the SysTick IRQ priority 
            if (TickPriority < (1UL << __NVIC_PRIO_BITS))
            {
                // Configure the TIM IRQ priority 
                HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, TickPriority, 0U);
                uwTickPrio = TickPriority;
            }
            else
            {
                status = HAL_ERROR;
            }
        }
    }

    return status;
}


/**
 * @brief Suspend Tick increment 
 * 
 * @details Disable the tick increment by disabling TIM11 update interrupt 
 */
void HAL_SuspendTick(void)
{
    // Disable TIM11 update Interrupt 
    __HAL_TIM_DISABLE_IT(&htim11, TIM_IT_UPDATE);
}


/**
 * @brief Resume Tick increment 
 * 
 * @details Enable the tick increment by Enabling TIM11 update interrupt.
 */
void HAL_ResumeTick(void)
{
    // Enable TIM11 Update interrupt 
    __HAL_TIM_ENABLE_IT(&htim11, TIM_IT_UPDATE);
}


/**
 * @brief Period elapsed callback in non blocking mode 
 * 
 * @details This function is called by HAL_TIM_IRQHandler which is called by the TIM11 
 *          interrupt handler. It makes a direct call to HAL_IncTick() to increment a 
 *          global variable "uwTick" used as application time base. 
 * 
 * @param  htim : TIM handle
 * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM11) 
    {
        HAL_IncTick();
    }
}

#endif   // FREERTOS_ENABLE 

//=======================================================================================
