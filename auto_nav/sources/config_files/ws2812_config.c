/**
 * @file ws2812_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief WS2812 LEDs (Neopixels) configuration 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "ws2812_config.h"

//=======================================================================================


//=======================================================================================
// LED colours 

// LED off 
const uint32_t ws2812_led_off = 0x00000000; 

// Autonomous state 
const uint32_t ws2812_led_auto_star = 0x001E0000;       // Starbird side indicator (green) 
const uint32_t ws2812_led_auto_port = 0x00001E00;       // Port side indicator (red) 
const uint32_t ws2812_led_auto_strobe = 0x001E1E1E;     // Strobe light (white) 

// Manual state 
const uint32_t ws2812_led_manual_strobe = 0x0000001E;   // Strobe light (blue) 

// Not ready state 
const uint32_t ws2812_led_not_ready = 0x00001E00;       // Not ready indicator (red) 

// Ready state 
const uint32_t ws2812_led_ready = 0x00FF6600;           // Ready indicator (orange) 

//=======================================================================================


//=======================================================================================
// Controller settings 

// LEDs used as strobe lights - 1-bit per LED (8 total) 
const uint8_t ws2812_strobe_leds = 0x3C; 
 // LED update software timer x this gives strobe period (s) 
const uint8_t ws2812_strobe_period = 10; 

//=======================================================================================
