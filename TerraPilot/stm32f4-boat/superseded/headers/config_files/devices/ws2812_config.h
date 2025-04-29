/**
 * @file ws2812_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief WS2812 LEDs (Neopixels) configuration interface 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _WS2812_CONFIG_H_ 
#define _WS2812_CONFIG_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "stm32f411xe.h"

//=======================================================================================


//=======================================================================================
// LED colours 

// LED off 
extern const uint32_t ws2812_led_off; 

extern const uint32_t ws2812_led_standby_not_ready; 
extern const uint32_t ws2812_led_standby_ready; 

// Autonomous state 
extern const uint32_t ws2812_led_auto_star;       // Starbird side indicator 
extern const uint32_t ws2812_led_auto_port;       // Port side indicator 
extern const uint32_t ws2812_led_auto_strobe;     // Strobe light 

// Manual control state 
extern const uint32_t ws2812_led_manual_strobe; 

// Low power state 
extern const uint32_t ws2812_led_low_pwr; 

// Not ready state 
extern const uint32_t ws2812_led_not_ready; 

// Ready state 
extern const uint32_t ws2812_led_ready; 

//=======================================================================================


//=======================================================================================
// Controller settings 

// LEDs used as strobe lights - 1-bit per LED (8 total) 
extern const uint8_t ws2812_strobe_leds; 
 // LED update software timer x this gives strobe period (s) 
extern const uint8_t ws2812_strobe_period; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _WS2812_CONFIG_H_ 
