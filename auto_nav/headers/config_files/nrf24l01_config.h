/**
 * @file nrf24l01_config.h
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 RF module configuration file interface 
 * 
 * @version 0.1
 * @date 2024-03-01
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef _NRF24L01_CONFIG_H_ 
#define _NRF24L01_CONFIG_H_ 

//=======================================================================================
// Includes 

#include "nrf24l01_driver.h" 

//=======================================================================================


//=======================================================================================
// RF pipe addresses 

// Address sent by the PTX and address accepted by the PRX 
extern const uint8_t nrf24l01_pipe_addr[NRF24l01_ADDR_WIDTH]; 

//=======================================================================================

#endif   // _NRF24L01_CONFIG_H_ 
