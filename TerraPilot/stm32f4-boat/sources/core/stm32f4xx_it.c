/**
 * @file stm32f4xx_it.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Interrupt Service Routines (ISRs) 
 * 
 * @version 0.1
 * @date 2024-03-04
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h" 

#if FREERTOS_ENABLE 
#include "stm32f4xx_hal_tim.h" 
#endif   // FREERTOS_ENABLE 

//=======================================================================================


//=======================================================================================
// Macros 

#define __weak __attribute__((weak)) 

//=======================================================================================


//=======================================================================================
// Globals 

// Instance of interrupt flag data record 
int_handle_flags_t handler_flags; 

// TIM11 HAL timer handler - when FreeRTOS needs SysTick 
#if FREERTOS_ENABLE 
extern TIM_HandleTypeDef htim11; 
#endif   // FREERTOS_ENABLE 

//=======================================================================================


//=======================================================================================
// Initialization 

// Interrupt handler flag initialization 
void int_handler_init(void)
{
    // Clear all flags 
    memset((void *)&handler_flags, CLEAR, sizeof(int_handle_flags_t)); 
}

//=======================================================================================


//=======================================================================================
// Cortex-M4 Processor Interruption and Exception Handlers 

// Non-maskable interrupt handler 
void NMI_Handler(void)
{
    while (1) {}
}

// Hard fault interrupt handler 
void HardFault_Handler(void)
{
    while (1) {}
}

// Memory management fault handler 
void MemManage_Handler(void)
{
    while (1) {}
}

// Pre-fetch fault, memory access fault handler 
void BusFault_Handler(void)
{
    while (1) {}
}

// Undefined instruction or illegal state handler 
void UsageFault_Handler(void)
{
    while (1) {}
}

// Debug monitor handler 
void DebugMon_Handler(void)
{
    // 
}

#if !FREERTOS_ENABLE 

// This function handles System service call via SWI instruction 
void SVC_Handler(void)
{
    // 
}

// This function handles Pendable request for system service 
void PendSV_Handler(void)
{
    // 
}

// This function handles System tick timer 
void SysTick_Handler(void)
{
    // HAL timer counter increment 
    HAL_IncTick(); 
}

#endif   // !FREERTOS_ENABLE 

//=======================================================================================


//=======================================================================================
// STM32F4xx Peripheral Interrupt Handlers 

// Interrupt handler names are defined in startup_stm32f411xe.s 

// EXTI Line 0 
__weak void EXTI0_IRQHandler(void)
{
    handler_flags.exti0_flag = SET_BIT; 
    exti_pr_clear(EXTI_L0); 
}


// EXTI Line 1 
__weak void EXTI1_IRQHandler(void)
{
    handler_flags.exti1_flag = SET_BIT; 
    exti_pr_clear(EXTI_L1);  
}


// EXTI Line 2 
__weak void EXTI2_IRQHandler(void)
{
    handler_flags.exti2_flag = SET_BIT; 
    exti_pr_clear(EXTI_L2); 
}


// EXTI Line 3 
__weak void EXTI3_IRQHandler(void)
{
    handler_flags.exti3_flag = SET_BIT; 
    exti_pr_clear(EXTI_L3); 
}


// EXTI Line 4 
__weak void EXTI4_IRQHandler(void)
{
    handler_flags.exti4_flag = SET_BIT; 
    exti_pr_clear(EXTI_L4); 
}


// EXTI lines 5-9 
__weak void EXTI9_5_IRQHandler(void)
{
    handler_flags.exti5_9_flag = SET_BIT; 
    exti_pr_clear(EXTI_L5 | EXTI_L6 | EXTI_L7 | EXTI_L8 | EXTI_L9); 
}


// EXTI lines 10-15 
__weak void EXTI15_10_IRQHandler(void)
{
    handler_flags.exti10_15_flag = SET_BIT; 
    exti_pr_clear(EXTI_L10 | EXTI_L11 | EXTI_L12 | EXTI_L13 | EXTI_L14 | EXTI_L15); 
}


// DMA1 Stream 0 
__weak void DMA1_Stream0_IRQHandler(void)
{
    handler_flags.dma1_0_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 1 
__weak void DMA1_Stream1_IRQHandler(void)
{
    handler_flags.dma1_1_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 2 
__weak void DMA1_Stream2_IRQHandler(void)
{
    handler_flags.dma1_2_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 3 
__weak void DMA1_Stream3_IRQHandler(void)
{
    handler_flags.dma1_3_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 4 
__weak void DMA1_Stream4_IRQHandler(void)
{
    handler_flags.dma1_4_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 5 
__weak void DMA1_Stream5_IRQHandler(void)
{
    handler_flags.dma1_5_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 6 
__weak void DMA1_Stream6_IRQHandler(void)
{
    handler_flags.dma1_6_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA1 Stream 7 
__weak void DMA1_Stream7_IRQHandler(void)
{
    handler_flags.dma1_7_flag = SET_BIT; 
    dma_clear_int_flags(DMA1); 
}


// DMA2 Stream 0 
__weak void DMA2_Stream0_IRQHandler(void)
{
    handler_flags.dma2_0_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 1 
__weak void DMA2_Stream1_IRQHandler(void)
{
    handler_flags.dma2_1_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 2 
__weak void DMA2_Stream2_IRQHandler(void)
{
    handler_flags.dma2_2_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 3 
__weak void DMA2_Stream3_IRQHandler(void)
{
    handler_flags.dma2_3_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 4 
__weak void DMA2_Stream4_IRQHandler(void)
{
    handler_flags.dma2_4_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 5 
__weak void DMA2_Stream5_IRQHandler(void)
{
    handler_flags.dma2_5_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 6 
__weak void DMA2_Stream6_IRQHandler(void)
{
    handler_flags.dma2_6_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// DMA2 Stream 7 
__weak void DMA2_Stream7_IRQHandler(void)
{
    handler_flags.dma2_7_flag = SET_BIT; 
    dma_clear_int_flags(DMA2); 
}


// Timer 1 break + timer 9 global 
__weak void TIM1_BRK_TIM9_IRQHandler(void)
{
    handler_flags.tim1_brk_tim9_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM1); 
    tim_uif_clear(TIM9); 
}


// Timer 1 update + timer 10 global 
__weak void TIM1_UP_TIM10_IRQHandler(void)
{
    handler_flags.tim1_up_tim10_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM1); 
    tim_uif_clear(TIM10); 
}

#if FREERTOS_ENABLE 

// Timer 1 trigger and communication + timer 11 global interrupts 
__weak void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim11); 
}

#else   // FREERTOS_ENABLE  

// Timer 1 trigger and communication + timer 11 global interrupts 
__weak void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
    handler_flags.tim1_trg_tim11_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM1); 
    tim_uif_clear(TIM11); 
}

#endif   // FREERTOS_ENABLE 

// Timer 1 capture compare 
__weak void TIM1_CC_IRQHandler(void)
{
    handler_flags.tim1_cc_flag = SET_BIT; 
    tim_uif_clear(TIM1); 
}


// Timer 2 
__weak void TIM2_IRQHandler(void)
{
    handler_flags.tim2_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM2); 
}


// Timer 3
__weak void TIM3_IRQHandler(void)
{
    handler_flags.tim3_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM3); 
}


// Timer 4
__weak void TIM4_IRQHandler(void)
{
    handler_flags.tim4_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM4); 
}


// Timer 5
__weak void TIM5_IRQHandler(void)
{
    handler_flags.tim5_glbl_flag = SET_BIT; 
    tim_uif_clear(TIM5); 
}


// ADC1 
__weak void ADC_IRQHandler(void)
{
    handler_flags.adc_flag = SET_BIT;  
}


// USART1 
__weak void USART1_IRQHandler(void)
{
    handler_flags.usart1_flag = SET_BIT; 
    dummy_read(USART1->SR); 
    dummy_read(USART1->DR); 
}


// USART2 
__weak void USART2_IRQHandler(void)
{
    handler_flags.usart2_flag = SET_BIT; 
    dummy_read(USART2->SR); 
    dummy_read(USART2->DR); 
}


// USART6 
__weak void USART6_IRQHandler(void)
{
    handler_flags.usart6_flag = SET_BIT; 
    dummy_read(USART6->SR); 
    dummy_read(USART6->DR); 
}

//=======================================================================================
