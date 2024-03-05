/**
 * @file stm32f4xx_it.h
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

#ifndef _STM32F4XX_IT_H_
#define _STM32F4XX_IT_H_

#ifdef __cplusplus
 extern "C" {
#endif

//=======================================================================================
// Includes 
//=======================================================================================


//=======================================================================================
// Prototypes 

void NMI_Handler(void); 
void HardFault_Handler(void); 
void MemManage_Handler(void); 
void BusFault_Handler(void); 
void UsageFault_Handler(void); 
void DebugMon_Handler(void); 

void SVC_Handler(void); 
void PendSV_Handler(void); 
void SysTick_Handler(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _STM32F4XX_IT_H_ 
