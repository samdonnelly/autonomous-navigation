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
extern uint32_t ws2812_led_off; 

// Autonomous state 
extern uint32_t ws2812_led_auto_star;       // Starbird side indicator 
extern uint32_t ws2812_led_auto_port;       // Port side indicator 
extern uint32_t ws2812_led_auto_strobe;     // Strobe light 

// Manual control state 
extern uint32_t ws2812_led_manual_strobe;   // Strobe light 

// Not ready state 
extern uint32_t ws2812_led_not_ready;       // Not ready indicator 

// Ready state 
extern uint32_t ws2812_led_ready;           // Ready indicator 

//=======================================================================================

#endif   // _WS2812_CONFIG_H_ 
