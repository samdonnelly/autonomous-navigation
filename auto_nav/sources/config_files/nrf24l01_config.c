/**
 * @file nrf24l01_config.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 RF module configuration file implementation 
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
// RF pipe addresses 

// Address sent by the PTX and address accepted by the PRX 
const uint8_t nrf24l01_pipe_addr[NRF24l01_ADDR_WIDTH] = 
{
    0xB3, 0xB4, 0xB5, 0xB6, 0x05
}; 

//=======================================================================================
