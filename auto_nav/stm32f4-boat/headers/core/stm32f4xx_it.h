/**
 * @file stm32f4xx_it.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Interrupt Service Routines (ISRs) interface 
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

#include "hardware_config.h" 
#include "includes_drivers.h" 

//=======================================================================================


//=======================================================================================
// Globals 

// Interrupt flag data record 
typedef struct int_handle_flags_s
{
    // EXTI interrupt flags 
    uint8_t exti0_flag     : 1;                    // Line 0 
    uint8_t exti1_flag     : 1;                    // Line 1 
    uint8_t exti2_flag     : 1;                    // Line 2 
    uint8_t exti3_flag     : 1;                    // Line 3 
    uint8_t exti4_flag     : 1;                    // Line 4 
    uint8_t exti5_9_flag   : 1;                    // Lines 5-9  
    uint8_t exti10_15_flag : 1;                    // Lines 10-15 

    // DMA1 interrupt flags 
    uint8_t dma1_0_flag : 1;                       // Stream 0 
    uint8_t dma1_1_flag : 1;                       // Stream 1 
    uint8_t dma1_2_flag : 1;                       // Stream 2 
    uint8_t dma1_3_flag : 1;                       // Stream 3 
    uint8_t dma1_4_flag : 1;                       // Stream 4 
    uint8_t dma1_5_flag : 1;                       // Stream 5 
    uint8_t dma1_6_flag : 1;                       // Stream 6 
    uint8_t dma1_7_flag : 1;                       // Stream 7 

    // DMA2 interrupt flags 
    uint8_t dma2_0_flag : 1;                       // Stream 0 
    uint8_t dma2_1_flag : 1;                       // Stream 1 
    uint8_t dma2_2_flag : 1;                       // Stream 2 
    uint8_t dma2_3_flag : 1;                       // Stream 3 
    uint8_t dma2_4_flag : 1;                       // Stream 4 
    uint8_t dma2_5_flag : 1;                       // Stream 5 
    uint8_t dma2_6_flag : 1;                       // Stream 6 
    uint8_t dma2_7_flag : 1;                       // Stream 7 

    // Timer interrupt flags 
    uint8_t tim1_brk_tim9_glbl_flag  : 1;          // TIM1 break + TIM9 global 
    uint8_t tim1_up_tim10_glbl_flag  : 1;          // TIM1 update + TIM10 global 
    uint8_t tim1_trg_tim11_glbl_flag : 1;          // TIM1 trigger, communication and global 
    uint8_t tim1_cc_flag             : 1;          // TIM1 capture compare 
    uint8_t tim2_glbl_flag           : 1;          // TIM2 global 
    uint8_t tim3_glbl_flag           : 1;          // TIM3 global 
    uint8_t tim4_glbl_flag           : 1;          // TIM4 global 
    uint8_t tim5_glbl_flag           : 1;          // TIM5 global 

    // ADC interrupt flags 
    uint8_t adc_flag: 1; 

    // USART interrupt flags 
    uint8_t usart1_flag : 1;                       // USART1 global 
    uint8_t usart2_flag : 1;                       // USART2 global 
    uint8_t usart6_flag : 1;                       // USART6 global 
} 
int_handle_flags_t;


// Instance of interrupt flag data record - defined for external use 
extern int_handle_flags_t handler_flags; 

//=======================================================================================


//=======================================================================================
// Initialization 

/**
 * @brief Interrupt handler flag initialization 
 * 
 * @details Clears all the interrupt flags in int_handle_flags_t so no interrupts are 
 *          triggered immediately after being enabled. 
 * 
 * @see int_handle_flags_t
 */
void int_handler_init(void); 

//=======================================================================================


//=======================================================================================
// System Handlers 

/**
 * @brief Non-maskable interrupt handler 
 */
void NMI_Handler(void); 


/**
 * @brief Hard fault interrupt handler 
 */
void HardFault_Handler(void); 


/**
 * @brief Memory management fault handler 
 */
void MemManage_Handler(void); 


/**
 * @brief Pre-fetch fault, memory access fault handler 
 */
void BusFault_Handler(void); 


/**
 * @brief Undefined instruction or illegal state handler 
 */
void UsageFault_Handler(void); 


/**
 * @brief Debug monitor handler 
 */
void DebugMon_Handler(void); 

#if !FREERTOS_ENABLE 

/**
 * @brief This function handles System service call via SWI instruction 
 */
void SVC_Handler(void); 


/**
 * @brief This function handles Pendable request for system service 
 */
void PendSV_Handler(void); 


/**
 * @brief This function handles System tick timer 
 */
void SysTick_Handler(void); 

#endif   // !FREERTOS_ENABLE 

//=======================================================================================


//=======================================================================================
// Peripheral Handlers 

/**
 * @brief EXTI Line 0 interrupt handler 
 * 
 * @details External interrupt handler for pin 0 of whichever port has been configured. This 
 *          function sets exti0_flag and clears it's corresponding bit in the pending 
 *          register. External interrupts are triggered on rising and/or falling edges of 
 *          GPIO input pins. This interrupt handler can also be configured to trigger for 
 *          software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI0_IRQHandler(void); 


/**
 * @brief EXTI Line 1 interrupt handler 
 * 
 * @details External interrupt handler for pin 1 of whichever port has been configured. This 
 *          function sets exti1_flag and clears it's corresponding bit in the pending 
 *          register. External interrupts are triggered on rising and/or falling edges of 
 *          GPIO input pins. This interrupt handler can also be configured to trigger for 
 *          software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI1_IRQHandler(void); 


/**
 * @brief EXTI Line 2 interrupt handler 
 * 
 * @details External interrupt handler for pin 2 of whichever port has been configured. This 
 *          function sets exti2_flag and clears it's corresponding bit in the pending 
 *          register. External interrupts are triggered on rising and/or falling edges of 
 *          GPIO input pins. This interrupt handler can also be configured to trigger for 
 *          software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI2_IRQHandler(void); 


/**
 * @brief EXTI Line 3 interrupt handler 
 * 
 * @details External interrupt handler for pin 3 of whichever port has been configured. This 
 *          function sets exti3_flag and clears it's corresponding bit in the pending 
 *          register. External interrupts are triggered on rising and/or falling edges of 
 *          GPIO input pins. This interrupt handler can also be configured to trigger for 
 *          software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI3_IRQHandler(void); 


/**
 * @brief EXTI Line 4 interrupt handler 
 * 
 * @details External interrupt handler for pin 4 of whichever port has been configured. This 
 *          function sets exti4_flag and clears it's corresponding bit in the pending 
 *          register. External interrupts are triggered on rising and/or falling edges of 
 *          GPIO input pins. This interrupt handler can also be configured to trigger for 
 *          software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI4_IRQHandler(void); 


/**
 * @brief EXTI lines 5-9 interrupt handler 
 * 
 * @details External interrupt handler for pins 5-9 of whichever port has been configured. This 
 *          means pins 5-9 will share this interrupt handler and it therefore won't distinguish 
 *          which pin has triggered the interrupt. This function sets exti5_9_flag and clears all 
 *          the bits corresponding to pins 5-9 in the pending register. External interrupts are 
 *          triggered on rising and/or falling edges of GPIO input pins. This interrupt handler 
 *          can also be configured to trigger for software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI9_5_IRQHandler(void); 


/**
 * @brief EXTI lines 10-15 interrupt handler 
 * 
 * @details External interrupt handler for pins 10-15 of whichever port has been configured. This 
 *          means pins 10-15 will share this interrupt handler and it therefore won't distinguish 
 *          which pin has triggered the interrupt. This function sets exti10_15_flag and clears all 
 *          the bits corresponding to pins 10-15 in the pending register. External interrupts are 
 *          triggered on rising and/or falling edges of GPIO input pins. This interrupt handler 
 *          can also be configured to trigger for software events. 
 * 
 * @see exti_pr_clear
 */
void EXTI15_10_IRQHandler(void); 


/**
 * @brief DMA1 Stream 0 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 0. This function sets dma1_0_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream0_IRQHandler(void); 


/**
 * @brief DMA1 Stream 1 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 1. This function sets dma1_1_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream1_IRQHandler(void); 


/**
 * @brief DMA1 Stream 2 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 2. This function sets dma1_2_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream2_IRQHandler(void); 


/**
 * @brief DMA1 Stream 3 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 3. This function sets dma1_3_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream3_IRQHandler(void); 


/**
 * @brief DMA1 Stream 4 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 4. This function sets dma1_4_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream4_IRQHandler(void); 


/**
 * @brief DMA1 Stream 5 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 5. This function sets dma1_5_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream5_IRQHandler(void); 


/**
 * @brief DMA1 Stream 6 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 6. This function sets dma1_6_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream6_IRQHandler(void); 


/**
 * @brief DMA1 Stream 7 interrupt handler 
 * 
 * @details Interrupt handler for DMA 1, stream 7. This function sets dma1_7_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA1_Stream7_IRQHandler(void); 


/**
 * @brief DMA2 Stream 0 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 0. This function sets dma2_0_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream0_IRQHandler(void); 


/**
 * @brief DMA2 Stream 1 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 1. This function sets dma2_1_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream1_IRQHandler(void); 


/**
 * @brief DMA2 Stream 2 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 2. This function sets dma2_2_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream2_IRQHandler(void); 


/**
 * @brief DMA2 Stream 3 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 3. This function sets dma2_3_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream3_IRQHandler(void); 


/**
 * @brief DMA2 Stream 4 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 4. This function sets dma2_4_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream4_IRQHandler(void); 


/**
 * @brief DMA2 Stream 5 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 5. This function sets dma2_5_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream5_IRQHandler(void); 


/**
 * @brief DMA2 Stream 6 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 6. This function sets dma2_6_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream6_IRQHandler(void); 


/**
 * @brief DMA2 Stream 7 interrupt handler 
 * 
 * @details Interrupt handler for DMA 2, stream 7. This function sets dma2_7_flag and 
 *          then clears all the DMA interrupt flags so that the handler can be exited. 
 *          Interrupts can be produced when half-transfer is reached, transfer is complete, 
 *          there is a transfer error, there is a FIFO error (overrun, underrun, FIFO level 
 *          error), or there is a direct mode error. 
 * 
 * @see dma_clear_int_flags
 */
void DMA2_Stream7_IRQHandler(void); 


/**
 * @brief Timer 1 break + timer 9 global interrupt handler 
 * 
 * @details Interrupt handler for both TIM1 break input and TIM9. 
 *          
 *          The break input is only for TIM1, an advanced-control timer, and it is used to 
 *          alert the software when a failure is detected in the HSE clock. In this case 
 *          the HSE oscillator is disabled and the interrupt is generated once the clock 
 *          failure event is sent to the break input. This can be used to perform safety 
 *          operations and put the timers output signals into xreset state or a known state. 
 *          
 *          TIM9 is a general purpose timer and can be set to trigger interrupts for a counter 
 *          update, capture/compare, or a trigger. This handler clears the update interrupt 
 *          flag for the timer so the handler can be exited. 
 *          
 *          This handler sets tim1_brk_tim9_glbl_flag and clears the update interrupt flag 
 *          for the timers so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM1_BRK_TIM9_IRQHandler(void); 


/**
 * @brief Timer 1 update + timer 10 global interrupt handler 
 * 
 * @details Interrupt handler for both TIM1 update and TIM10 global. 
 *          
 *          TIM1 is an advanced timer and this handler is specifically for TIM1 updates. Updates 
 *          include counter overflow/underflow or counter initialization. 
 *          
 *          TIM10 is a general purpose timer and can be set to trigger interrupts for counter 
 *          updates (overflow, initialization), input capture or output compare. 
 *          
 *          This handler sets tim1_up_tim10_glbl_flag and clears the update interrupt flag 
 *          for the timers so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM1_UP_TIM10_IRQHandler(void); 


#if !FREERTOS_ENABLE 
/**
 * @brief Timer 1 trigger and communication + timer 11 global interrupt handler 
 * 
 * @details Interrupt handler for both TIM1 tigger and TIM11 global. 
 *          
 *          This handler can be called from TIM1 trigger events such as counter start, stop, 
 *          or initialization, or count by internal/external trigger. 
 *          
 *          TIM11 is a general purpose timer and can be set to trigger interrupts for counter 
 *          updates (overflow, initialization), input capture or output compare. 
 *          
 *          This handler sets tim1_trg_tim11_glbl_flag and clears the update interrupt flag 
 *          for the timers so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM1_TRG_COM_TIM11_IRQHandler(void); 
#endif   // FREERTOS_ENABLE 


/**
 * @brief Timer 1 capture compare interrupt handler 
 * 
 * @details Interrupt handler for TIM1 input capture and output compare. 
 *          
 *          Input capture is used to latch the counter value when a transition is detected in 
 *          the corresponding ICx signal. This transition will trigger the interrupt. 
 *          
 *          Output compare is used to control an output waveform (such as PWM) or indicating 
 *          when a period of time has elapsed. This interrupt is triggered when a match is 
 *          found between the capture/compare register and the counter. 
 *          
 *          This handler sets tim1_cc_flag and clears the update interrupt flag for the timer 
 *          so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM1_CC_IRQHandler(void); 


/**
 * @brief Timer 2 interrupt handler 
 * 
 * @details Interrupt handler for TIM2. This handler can be called for update events 
 *          (counter overflow/underflow or counter initialization), trigger events (counter 
 *          start, stop, initialization or count by internal/external trigger), input 
 *          capture or output compare. 
 *          
 *          This handler sets tim2_glbl_flag and clears the update interrupt flag for the timer 
 *          so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM2_IRQHandler(void); 


/**
 * @brief Timer 3 interrupt handler 
 * 
 * @details Interrupt handler for TIM3. This handler can be called for update events 
 *          (counter overflow/underflow or counter initialization), trigger events (counter 
 *          start, stop, initialization or count by internal/external trigger), input 
 *          capture or output compare. 
 *          
 *          This handler sets tim3_glbl_flag and clears the update interrupt flag for the timer 
 *          so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM3_IRQHandler(void); 


/**
 * @brief Timer 4 interrupt handler 
 * 
 * @details Interrupt handler for TIM4. This handler can be called for update events 
 *          (counter overflow/underflow or counter initialization), trigger events (counter 
 *          start, stop, initialization or count by internal/external trigger), input 
 *          capture or output compare. 
 *          
 *          This handler sets tim4_glbl_flag and clears the update interrupt flag for the timer 
 *          so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM4_IRQHandler(void); 


/**
 * @brief Timer 5 interrupt handler 
 * 
 * @details Interrupt handler for TIM5. This handler can be called for update events 
 *          (counter overflow/underflow or counter initialization), trigger events (counter 
 *          start, stop, initialization or count by internal/external trigger), input 
 *          capture or output compare. 
 *          
 *          This handler sets tim5_glbl_flag and clears the update interrupt flag for the timer 
 *          so the handler can be exited. 
 * 
 * @see tim_uif_clear
 */
void TIM5_IRQHandler(void); 


/**
 * @brief ADC1 interrupt handler 
 * 
 * @details Interrupt handler for ADC1. This handler sets adc_flag. Interrupt generation for 
 *          ADC1 can be configured to trigger at the end of conversion, end of injected 
 *          conversion, and for analog watchdog or overrun events. 
 */
void ADC_IRQHandler(void); 


/**
 * @brief USART1 interrupt handler 
 * 
 * @details Interrupt handler for USART1. This handler sets usart1_flag. 
 */
void USART1_IRQHandler(void); 


/**
 * @brief USART2 interrupt handler 
 * 
 * @details Interrupt handler for USART2. This handler sets usart2_flag. 
 */
void USART2_IRQHandler(void); 


/**
 * @brief USART6 interrupt handler 
 * 
 * @details Interrupt handler for USART6. This handler sets usart6_flag. 
 */
void USART6_IRQHandler(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _STM32F4XX_IT_H_ 
