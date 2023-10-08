/**
 * @file project_init.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief Autonomous boat initialization header 
 * 
 * @version 0.1
 * @date 2023-07-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _AB_INIT_H_ 
#define _AB_INIT_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "ab_includes_app.h"
#include "includes_drivers.h"

//=======================================================================================


//=======================================================================================
// Macros 

// Device setup 
#define AB_IMU_STBY_MASK 0x00        // Axis standby status mask 
#define AB_IMU_SMPLRT_DIVIDER 0      // Sample Rate Divider 
#define AB_MAG_NUM_DIRS 8            // Number of directions of heading offset calcs 
#define AB_RF_CH_FREQ 10             // Frequency channel for the RF module 
#define AB_ESC_PERIOD 20000          // ESC PWM timer period (auto-reload register) 
#define AB_ESC_FWD_SPEED_LIM 1600    // Forward PWM pulse time limit (us) 
#define AB_ESC_REV_SPEED_LIM 1440    // Reverse PWM pulse time limit (us) 

//=======================================================================================


//=======================================================================================
// Functions 

/**
 * @brief Autonomous boat initialization 
 * 
 * @details 
 */
void ab_init(void); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _AB_INIT_H_ 
