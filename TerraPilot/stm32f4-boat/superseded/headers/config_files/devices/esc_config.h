/**
 * @file esc_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief ESC module configuration interface 
 * 
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _ESC_CONFIG_H_ 
#define _ESC_CONFIG_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "esc_driver.h" 

//=======================================================================================


//=======================================================================================
// Macros 

#define ESC_NO_THRUST 0   // Force thruster output to zero 

//=======================================================================================


//=======================================================================================
// Parameters 

// PWM timer period (auto-reload register) 
extern const uint16_t esc_period; 

// PWM limits (us) 
extern const uint16_t esc1_fwd_speed_lim; 
extern const uint16_t esc1_rev_speed_lim; 
extern const uint16_t esc2_fwd_speed_lim; 
extern const uint16_t esc2_rev_speed_lim; 

extern const uint16_t esc_fwd_speed_lim; 
extern const uint16_t esc_rev_speed_lim; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _ESC_CONFIG_H_ 
