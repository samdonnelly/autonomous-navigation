/**
 * @file nrf24l01_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 module configuration 
 * 
 * @version 0.1
 * @date 2024-03-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "nrf24l01_config.h" 

//=======================================================================================


//=======================================================================================
// Data 

// RF pipe address sent by the PTX and address accepted by the PRX 
const uint8_t nrf24l01_pipe_addr[NRF24l01_ADDR_WIDTH] = 
{
    0xB3, 0xB4, 0xB5, 0xB6, 0x05
}; 

// Data pipe number 
const nrf24l01_data_pipe_t nrf24l01_pipe = NRF24L01_DP_1; 

// Frequency channel for the RF module 
const uint8_t nrf24l01_rf_channel_freq = 10; 

//=======================================================================================
