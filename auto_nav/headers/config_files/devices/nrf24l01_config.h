/**
 * @file nrf24l01_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 module configuration interface 
 * 
 * @version 0.1
 * @date 2024-03-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _NRF24L01_CONFIG_H_ 
#define _NRF24L01_CONFIG_H_ 

#ifdef __cplusplus
extern "C" {
#endif

//=======================================================================================
// Includes 

#include "nrf24l01_driver.h" 

//=======================================================================================


//=======================================================================================
// Data 

// RF pipe address sent by the PTX and address accepted by the PRX 
extern const uint8_t nrf24l01_pipe_addr[NRF24l01_ADDR_WIDTH]; 

// Data pipe number 
extern const nrf24l01_data_pipe_t nrf24l01_pipe; 

// Frequency channel for the RF module 
extern const uint8_t nrf24l01_rf_channel_freq; 

//=======================================================================================

#ifdef __cplusplus
}
#endif

#endif   // _NRF24L01_CONFIG_H_ 
