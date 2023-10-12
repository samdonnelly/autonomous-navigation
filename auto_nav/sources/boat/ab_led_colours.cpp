/**
 * @file ab_led_colours.cpp
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief AB LED colour data 
 * 
 * @version 0.1
 * @date 2023-10-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */

//=======================================================================================
// Includes 

#include "ab_led_colours.h"

//=======================================================================================


//=======================================================================================
// LED colours 

// LED off 
uint32_t ab_led_clear = 0x00000000; 

// LED 0 colours 
uint32_t ab_led0_auto_star = 0x001E0000;    // Autonomous state - Starbird side (green) 

// LED 1 colours 
uint32_t ab_led1_auto_star = 0x001E0000;    // Autonomous state - Starbird side (green) 

// LED 2 colours 

// Strobe LEDs (LEDs 3 and 4) 
uint32_t ab_led_not_ready = 0x00001E00;     // Not ready state - indicator (red) 
uint32_t ab_led_ready = 0x00FF6600;         // Ready state - indicator (orange) 
uint32_t ab_led_manual = 0x0000001E;        // Manual control state - Strobe (blue) 
uint32_t ab_led_auto = 0x001E1E1E;          // Autonomous state - Strobe (white) 

// LED 5 colours 

// LED 6 colours 
uint32_t ab_led6_auto_port = 0x00001E00;    // Autonomous state - Port side (red) 

// LED 7 colours 
uint32_t ab_led7_auto_port = 0x00001E00;    // Autonomous state - Port side (red) 

//=======================================================================================