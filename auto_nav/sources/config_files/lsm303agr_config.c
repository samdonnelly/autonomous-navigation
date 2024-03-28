/**
 * @file lsm303agr_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief LSM303AGR module configuration 
 * 
 * @version 0.1
 * @date 2024-01-31
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "lsm303agr_config.h" 

//=======================================================================================


//=======================================================================================
// Data 

// Magnetometer directional offsets to correct for heading errors (units: degrees*10) 
const int16_t lsm303agr_config_dir_offsets[LSM303AGR_M_NUM_DIR] = 
{
    -160,     // N  (0/360deg) 
    32,       // NE (45deg) 
    215,      // E  (90deg) 
    385,      // SE (135deg) 
    435,      // S  (180deg) 
    20,       // SW (225deg) 
    -450,     // W  (270deg) 
    -365      // NW (315deg) 
}; 

// Heading calculation low pass filter gain 
const double lsm303agr_lpf_gain = 0.2; 

//=======================================================================================
