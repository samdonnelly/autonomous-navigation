/**
 * @file nrf24l01_driver_mock.c
 * 
 * @author Sam Donnelly (samueldonnelly11@gmail.com)
 * 
 * @brief nRF24L01 driver mock 
 * 
 * @version 0.1
 * @date 2024-04-12
 * 
 * @copyright Copyright (c) 2024
 * 
 */

//=======================================================================================
// Includes 

#include "nrf24l01_driver.h" 

//=======================================================================================


//=======================================================================================
// Driver functions 

// nRF24L01 initialization 
void nrf24l01_init(
    SPI_TypeDef *spi, 
    GPIO_TypeDef *gpio_ss, 
    pin_selector_t ss_pin, 
    GPIO_TypeDef *gpio_en, 
    pin_selector_t en_pin, 
    TIM_TypeDef *timer)
{
    // 
}


// Configure a devices PTX settings 
void nrf24l01_ptx_config(const uint8_t *tx_addr)
{
    // 
}


// Configure a devices PRX settings 
void nrf24l01_prx_config(
    const uint8_t *rx_addr, 
    nrf24l01_data_pipe_t pipe_num)
{
    // 
}


// Set RF channel 
void nrf24l01_set_rf_channel(uint8_t rf_ch_freq)
{
    // 
}


// Set data rate 
void nrf24l01_set_rf_dr(nrf24l01_data_rate_t rate)
{
    // 
}


// Set power output 
void nrf24l01_set_rf_pwr(nrf24l01_rf_pwr_t rf_pwr)
{
    // 
}


// Power down 
void nrf24l01_pwr_down(void)
{
    // 
}


// Power up 
void nrf24l01_pwr_up(void)
{   
    // 
}


// Data ready status 
uint8_t nrf24l01_data_ready_status(nrf24l01_data_pipe_t pipe_num)
{
    return TRUE; 
}


// Get power mode 
nrf24l01_pwr_mode_t nrf24l01_get_pwr_mode(void)
{
    return NRF24L01_PWR_UP; 
}


// Get active mode 
nrf24l01_mode_select_t nrf24l01_get_mode(void)
{
    return NRF24L01_TX_MODE; 
}


// Get RF channel 
uint8_t nrf24l01_get_rf_ch(void)
{
    return TRUE; 
}


// Get RF data rate 
nrf24l01_data_rate_t nrf24l01_get_rf_dr(void)
{
    return NRF24L01_DR_1MBPS; 
}


// Get power output 
nrf24l01_rf_pwr_t nrf24l01_get_rf_pwr(void)
{
    return NRF24L01_RF_PWR_18DBM; 
}


// Receive payload 
void nrf24l01_receive_payload(
    uint8_t *read_buff, 
    nrf24l01_data_pipe_t pipe_num)
{
    // 
}


// Send payload 
uint8_t nrf24l01_send_payload(const uint8_t *data_buff)
{
    return TRUE; 
}

//=======================================================================================
