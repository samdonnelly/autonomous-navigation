/**
 * @file lsm303agr_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief LSM303AGR module configuration interface 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _LSM303AGR_CONFIG_H_ 
#define _LSM303AGR_CONFIG_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "lsm303agr_driver.h" 

//=======================================================================================


//=======================================================================================
// Data 

// Magnetometer directional offsets to correct for heading errors (units: degrees*10) 
extern const int16_t lsm303agr_config_dir_offsets[LSM303AGR_M_NUM_DIR]; 

// Heading calculation low pass filter gain 
extern const double lsm303agr_lpf_gain; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _LSM303AGR_CONFIG_H_ 
