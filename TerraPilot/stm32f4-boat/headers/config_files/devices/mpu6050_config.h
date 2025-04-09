/**
 * @file mpu6050_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief MPU-6050 module configuration interface 
 * 
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _MPU6050_CONFIG_H_ 
#define _MPU6050_CONFIG_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "mpu6050_driver.h" 

//=======================================================================================


//=======================================================================================
// Device parameters 

// Axis standby status mask 
extern const uint8_t mpu6050_standby_mask; 

// Sample Rate Divider 
extern const MPU6050_SMPLRT_DIV mpu6050_sample_rate_divider; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _MPU6050_CONFIG_H_ 
