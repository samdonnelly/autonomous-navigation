/**
 * @file lsm303agr_driver_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief LSM303AGR driver mock 
 * 
 * @version 0.1
 * @date 2024-06-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "lsm303agr_driver_mock.h" 

//=======================================================================================


//=======================================================================================
// Macros 
//=======================================================================================


//=======================================================================================
// Variables 
//=======================================================================================


//=======================================================================================
// Driver functions 

// Magnetometer initialization 
LSM303AGR_STATUS lsm303agr_m_init(
    I2C_TypeDef *i2c, 
    const int16_t *offsets, 
    double heading_lpf_gain, 
    lsm303agr_m_odr_cfg_t m_odr, 
    lsm303agr_m_sys_mode_t m_mode, 
    lsm303agr_cfg_t m_off_canc, 
    lsm303agr_cfg_t m_lpf, 
    lsm303agr_cfg_t m_int_mag_pin, 
    lsm303agr_cfg_t m_int_mag)
{
    return 0; 
}


// Calibrate the magnetometer heading 
void lsm303agr_m_heading_calibration(const int16_t *offsets)
{
    // 
}


// Read the most recent magnetometer data 
LSM303AGR_STATUS lsm303agr_m_update(void)
{
    return 0; 
}


// Get magnetometer axis data 
void lsm303agr_m_get_axis_data(int16_t *m_axis_data)
{
    // 
}


// Get magnetometer applied magnetic field reading for each axis 
void lsm303agr_m_get_field(int32_t *m_field_data)
{
    // 
}


// Get magnetometer (compass) heading 
int16_t lsm303agr_m_get_heading(void)
{
    return 0; 
}

//=======================================================================================


//=======================================================================================
// Mock functions 
//=======================================================================================
