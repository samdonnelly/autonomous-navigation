/**
 * @file lsm303agr_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief LSM303AGR 6-axis magnetometer and accelerometer configuration file interface 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _LSM303AGR_CONFIG_H_ 
#define _LSM303AGR_CONFIG_H_ 

//=======================================================================================
// Includes 

#include "lsm303agr_driver.h" 

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Config messages 

// Magnetometer directional offsets to correct for heading errors (units: degrees*10) 
extern const int16_t lsm303agr_config_dir_offsets[LSM303AGR_M_NUM_DIR]; 

//=======================================================================================

#endif   // _LSM303AGR_CONFIG_H_ 
