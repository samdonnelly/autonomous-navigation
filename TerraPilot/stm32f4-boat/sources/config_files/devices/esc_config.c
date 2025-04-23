/**
 * @file esc_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief ESC module configuration 
 * 
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "esc_config.h" 

//=======================================================================================


//=======================================================================================
// Parameters 

// PWM timer period (auto-reload register) 
const uint16_t esc_period = 20000; 

// PWM limits (us) 
const uint16_t esc1_fwd_speed_lim = 1750; 
const uint16_t esc1_rev_speed_lim = 1430; 
const uint16_t esc2_fwd_speed_lim = 1730; 
const uint16_t esc2_rev_speed_lim = 1430; 

const uint16_t esc_fwd_speed_lim = 2000; 
const uint16_t esc_rev_speed_lim = 1000; 

//=======================================================================================
