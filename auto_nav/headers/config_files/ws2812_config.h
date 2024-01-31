/**
 * @file ws2812_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief WS2812 LEDs (Neopixels) configuration file interface 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _WS2812_CONFIG_H_ 
#define _WS2812_CONFIG_H_ 

//=======================================================================================
// Includes 

#include "stm32f411xe.h"

//=======================================================================================


//=======================================================================================
// LED colours 

// LED off 
extern uint32_t ws2812_led_clear; 

// LED 0 colours 
extern uint32_t ws2812_led0_auto_star;    // Autonomous state - Starbird side 

// LED 1 colours 
extern uint32_t ws2812_led1_auto_star;    // Autonomous state - Starbird side 

// LED 2 colours 

// Strobe LEDs (LEDs 3 and 4) 
extern uint32_t ws2812_led_not_ready;     // Not ready state - indicator 
extern uint32_t ws2812_led_ready;         // Ready state - indicator 
extern uint32_t ws2812_led_manual;        // Manual control state - Strobe 
extern uint32_t ws2812_led_auto;          // Autonomous state - Strobe 

// LED 5 colours 

// LED 6 colours 
extern uint32_t ws2812_led6_auto_port;    // Autonomous state - Port side 

// LED 7 colours 
extern uint32_t ws2812_led7_auto_port;    // Autonomous state - Port side 

//=======================================================================================

#endif   // _WS2812_CONFIG_H_ 
