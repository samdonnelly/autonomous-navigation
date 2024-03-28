/**
 * @file mpu6050_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief MPU-6050 module configuration 
 * 
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "mpu6050_config.h" 

//=======================================================================================


//=======================================================================================
// Device parameters 

const uint8_t mpu6050_standby_mask = 0x00; 
const MPU6050_SMPLRT_DIV mpu6050_sample_rate_divider = 0; 

//=======================================================================================
