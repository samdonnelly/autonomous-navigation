/**
 * @file nrf24l01_driver_mock.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 driver mock interface 
 * 
 * @version 0.1
 * @date 2024-04-18
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _NRF24L01_DRIVER_MOCK_H_ 
#define _NRF24L01_DRIVER_MOCK_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "nrf24l01_driver.h" 

//=======================================================================================


//=======================================================================================
// Mock functions 

/**
 * @brief Set the data to be read 
 * 
 * @param read_buff_set : buffer containing the data to be read from the mock 
 * @param buff_size : size of the data 
 */
void nrf24l01_mock_set_read_data(
    uint8_t *read_buff_set, 
    uint8_t buff_size); 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _NRF24L01_DRIVER_MOCK_H_ 
