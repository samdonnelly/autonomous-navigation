/**
 * @file ws2812_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief WS2812 LEDs (Neopixels) configuration 
 *        
 *        Colour Palette: https://www.color-hex.com/color-palette/16853 
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

// Standby state 
const uint32_t ws2812_led_standby_not_ready = 0x0044EE00;   // Ready indicator (orange) 
const uint32_t ws2812_led_standby_ready = 0x0044EE00;       // Not ready indicator (orange) 

// Autonomous state 
const uint32_t ws2812_led_auto_star = 0x001E0000;           // Starbird side indicator (green) 
const uint32_t ws2812_led_auto_port = 0x00001E00;           // Port side indicator (red) 
const uint32_t ws2812_led_auto_strobe = 0x005E5E5E;         // Strobe light (white) 

// Manual state 
const uint32_t ws2812_led_manual_strobe = 0x000038FF;       // Strobe light (purple) 

// Low power state 
const uint32_t ws2812_led_low_pwr = 0x00FFC823;             // Low power (yellow) 

// Not ready state 
const uint32_t ws2812_led_not_ready = 0x00001E00;           // Not ready indicator (red) 

// Ready state 
const uint32_t ws2812_led_ready = 0x00FF6600;               // Ready indicator (orange) 

//=======================================================================================


//=======================================================================================
// Controller settings 

// LEDs used as strobe lights - 1-bit per LED (8 total) 
const uint8_t ws2812_strobe_leds = 0x3C; 
// LED update software timer x this gives strobe period (s) 
const uint8_t ws2812_strobe_period = 10; 

//=======================================================================================
