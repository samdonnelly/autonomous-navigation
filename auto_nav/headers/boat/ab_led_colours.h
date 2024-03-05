/**
 * @file ab_led_colours.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB LED colour data header 
 * 
 * @version 0.1
 * @date 2023-10-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _AB_LED_COLOURS_H_ 
#define _AB_LED_COLOURS_H_ 

//=======================================================================================
// Includes 

#include "stm32f411xe.h" 

//=======================================================================================


//=======================================================================================
// LED colours 

// LED off 
extern uint32_t ab_led_clear; 

// LED 0 colours 
extern uint32_t ab_led0_auto_star;    // Autonomous state - Starbird side 

// LED 1 colours 
extern uint32_t ab_led1_auto_star;    // Autonomous state - Starbird side 

// LED 2 colours 

// Strobe LEDs (LEDs 3 and 4) 
extern uint32_t ab_led_not_ready;     // Not ready state - indicator 
extern uint32_t ab_led_ready;         // Ready state - indicator 
extern uint32_t ab_led_manual;        // Manual control state - Strobe 
extern uint32_t ab_led_auto;          // Autonomous state - Strobe 

// LED 5 colours 

// LED 6 colours 
extern uint32_t ab_led6_auto_port;    // Autonomous state - Port side 

// LED 7 colours 
extern uint32_t ab_led7_auto_port;    // Autonomous state - Port side 

//=======================================================================================

#endif   // _AB_LED_COLOURS_H_ 
